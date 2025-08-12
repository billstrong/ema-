bl_info = {
    "name": "USF4 Animation Handler",
    "description": "Addon to load, preview, edit and save USF4/SFxT animations",
    "blender": (2, 92, 0),
    "version": (1, 0, 1),
    "category": "Animation"
}

import struct
import bpy, math, mathutils
from bpy.props import StringProperty, BoolProperty, EnumProperty, IntProperty
from bpy_extras.io_utils import ImportHelper, ExportHelper
import array
from mathutils import Vector,Matrix,Euler,Quaternion
from bpy.app.handlers import persistent
import json
from operator import itemgetter, attrgetter
import time

import sys
import os
#dir = os.path.dirname(bpy.data.filepath)
#if not dir in sys.path:
#    sys.path.append(dir )

import importlib

from .EMAReader import *
from .IKProcessing import *

importlib.reload(EMAReader)
importlib.reload(IKProcessing)

armature_list = []

#set up custom property
bpy.types.PoseBone.absolute_scale = bpy.props.BoolProperty(name="Absolute Scale", default=False)
bpy.types.PoseBone.absolute_rotation = bpy.props.BoolProperty(name="Absolute Rotation", default=False)
bpy.types.PoseBone.absolute_translation = bpy.props.BoolProperty(name="Absolute Translation", default=False)
bpy.types.PoseBone.animation_override = bpy.props.BoolProperty(name="Animation Override", default=False)
bpy.types.PoseBone.animated = bpy.props.BoolProperty(name="Animated", default=False)

def GetCurves(act, bone_name):
    fcurves_list = []
    group = act.groups.get(bone_name)
    if group == None:
        return []
    else:
        return group.channels

def GetArmatureData(name):
    global armature_list

    for ad in armature_list:
        if ad.ObjName == name:
            return ad
    else:
        return None

def EulerToQuat(euler):
    dpitch = euler.y
    dyaw = euler.z
    droll = euler.x

    dSinPitch = math.sin(dpitch * 0.5)
    dCosPitch = math.cos(dpitch * 0.5)
    dSinYaw = math.sin(dyaw * 0.5)
    dCosYaw = math.cos(dyaw * 0.5)
    dSinRoll = math.sin(droll * 0.5)
    dCosRoll = math.cos(droll * 0.5)
    dCosPitchCosYaw = dCosPitch * dCosYaw
    dSinPitchSinYaw = dSinPitch * dSinYaw

    rot = mathutils.Quaternion()
    
    rot.x = dSinRoll * dCosPitchCosYaw - dCosRoll * dSinPitchSinYaw
    rot.y = dCosRoll * dSinPitch * dCosYaw + dSinRoll * dCosPitch * dSinYaw
    rot.z = dCosRoll * dCosPitch * dSinYaw - dSinRoll * dSinPitch * dCosYaw
    rot.w = dCosRoll * dCosPitchCosYaw + dSinRoll * dSinPitchSinYaw
    
    return rot

def SetupFrame(ema, action):
#Loads default transform values for each bone, evaluates animation curves, and combines with the values as needed

    for n in ema.Skeleton.Nodes:
        #Skip nodes tagged as un-animated
        if n.BitFlag == 0:
            continue
        #Reset flags
        n.AnimatedMatrix = n.Matrix
        n.AnimatedLocalMatrix = n.Matrix
        n.AnimatedRotationQuaternion = n.RotationQuaternion
        n.AnimatedScale = n.Scale
        n.AnimatedTranslation = n.Translation
        
        #fetch curves
        b_curves = GetCurves(action, n.Name)
        
        #Set up default transform from the ema matrix
        loc = n.Translation
        rot = n.RotationQuaternion
        sca = n.Scale
        
        #print(n.ID,n.Name)
        #print(loc)

        #If we have curves, process them...
        if len(b_curves) > 0:

            #euler to store evaluated rotations until we can convert to quat
            temp_euler = mathutils.Euler((0,0,0), 'XYZ')
            
            #Evaluate curves
            for c in b_curves:
                if c.data_path.find(".location") != -1:
                    loc[c.array_index] = c.evaluate(bpy.context.scene.frame_current)
                elif c.data_path.find(".rotation_euler") != -1:
                    temp_euler[c.array_index] = c.evaluate(bpy.context.scene.frame_current)
                elif c.data_path.find(".scale") != -1:
                    sca[c.array_index] = c.evaluate(bpy.context.scene.frame_current)
                else:
                    print("Unknown transform type, discarded")
            
            #If we've got evaluated rotation, convert to quat and overwrite
            if temp_euler != mathutils.Euler():
                rot = EulerToQuat(temp_euler)
                  
        n.AnimatedTranslation = loc
        n.AnimatedRotationQuaternion = rot
        n.AnimatedScale = sca

def UpdateFrame(ema, arm):
    for n in ema.Skeleton.Nodes:
        #Skip nodes tagged as un-animated
        if n.BitFlag == 0:
            continue
            
        translation = n.AnimatedTranslation
        rotation = n.AnimatedRotationQuaternion
        scale = n.AnimatedScale
        
        parent = None
        #Multiply by parent transform if necessary
        if n.Parent != -1:
            parent = ema.Skeleton.Nodes[n.Parent]
            if arm.pose.bones.get(n.Name).absolute_translation == False:
                translation = parent.AnimatedMatrix @ translation
            if arm.pose.bones.get(n.Name).absolute_rotation == False:
                rotation = parent.AnimatedRotationQuaternion @ rotation
            if arm.pose.bones.get(n.Name).absolute_scale == False:
                scale.x = scale.x * round(parent.AnimatedScale.x,6)
                scale.y = scale.y * round(parent.AnimatedScale.y,6)
                scale.z = scale.z * round(parent.AnimatedScale.z,6)
        
        #Generate armature-space transform matrices
        loc_matrix = mathutils.Matrix.Translation(translation)
        rot_matrix = rotation.to_matrix().to_4x4()
        sca_matrix = sca_matrix = mathutils.Matrix(([scale.x,0,0,0],[0,scale.y,0,0],[0,0,scale.z,0],[0,0,0,1]))
        
        #And combine
        matrix = loc_matrix @ rot_matrix @ sca_matrix
        
        #Assign armature-space matrix and individual transform components
        n.AnimatedMatrix = matrix
        n.AnimatedTranslation = translation
        n.AnimatedRotationQuaternion = rotation
        n.AnimatedScale = scale
        
        #Calculate local matrix by multiplying armature-space transform with inverse parent matrix
        if parent is not None:
            n.AnimatedLocalMatrix = parent.AnimatedMatrix.inverted() @ matrix
        else:
            n.AnimatedLocalMatrix = matrix

def AssignMatrices(ema, arm):
    ##TODO fix this, or at least check that it's always valid...
    for n in ema.Skeleton.Nodes:
        #Skip "unmatched" bones (hopefully they don't matter...)
        if arm.pose.bones.get(n.Name) == None:
            continue
        if n.Name == "LLegEff":
            arm.pose.bones.get(n.Name).matrix = n.AnimatedMatrix
        elif n.Name == "RLegEff":
            arm.pose.bones.get(n.Name).matrix = n.AnimatedMatrix
        elif n.Name == "LLegUp":
            arm.pose.bones.get(n.Name).matrix = n.AnimatedMatrix
        elif n.Name == "RLegUp":
            arm.pose.bones.get(n.Name).matrix = n.AnimatedMatrix
        elif n.Name == "LArmEff":
            arm.pose.bones.get(n.Name).matrix = n.AnimatedMatrix
        elif n.Name == "LArmUp":
            arm.pose.bones.get(n.Name).matrix = n.AnimatedMatrix
        elif n.Name == "RArmEff":
            arm.pose.bones.get(n.Name).matrix = n.AnimatedMatrix
        elif n.Name == "RArmUp":
            arm.pose.bones.get(n.Name).matrix = n.AnimatedMatrix
        else:
            irestdae = n.SBPMatrix
            par = mathutils.Matrix.Translation(([0,0,0]))
            if n.Parent != -1:
                par = ema.Skeleton.Nodes[n.Parent].SBPMatrix.inverted()
            
            rest = arm.pose.bones.get(n.Name).bone.matrix_local
            mat_final = MatrixDirectXToBlender(n.AnimatedLocalMatrix, rest, irestdae, par)

            arm.pose.bones.get(n.Name).matrix_basis = mat_final

    bpy.context.view_layer.update()

@persistent
def EMAProcessing(scene):
    global armature_list

    for ad in armature_list:
        ema = ad.EMA
        arm = bpy.data.objects.get(ad.ObjName)
        
        if arm.animation_data is not None:
            action = arm.animation_data.action
        
        if ema is not None and action is not None:
            SetupFrame(ema, action)
            
            UpdateFrame(ema, arm)
            
            AssignMatrices(ema, arm)
 
@persistent
def IKProcessingHandler2(scene):
    global ema
    
    if ema is not None and armature is not None:
        
        for IKData in ema.Skeleton.IKData:
            if IKData.Method == 0x00 and IKData.Flag0x00 == 0x02:
                nodes = ((ema.Skeleton.Nodes[IKData.NodeIDs[0]],ema.Skeleton.Nodes[IKData.NodeIDs[1]],ema.Skeleton.Nodes[IKData.NodeIDs[2]],ema.Skeleton.Nodes[IKData.NodeIDs[3]],ema.Skeleton.Nodes[IKData.NodeIDs[4]]))
                
                result = ProcessIKData0x00_02(nodes, IKData.Flag0x01)
                
                result0_local = result[0] @ ema.Skeleton.Nodes[nodes[1].Parent].AnimatedMatrix.inverted()
                
                ##node2 is easy because the parent is node1, so we already have the parent world matrix
                result1_local = result[0].inverted() @ result[1]
                
                    
@persistent
def IKProcessingHandler(scene):
    global armature_list
    
    for ad in armature_list:
        if ad.EMA is not None:   
            ema = ad.EMA
            armature = bpy.data.objects[ad.ObjName]
            if armature.animation_data is not None:
                action = armature.animation_data.action

            if action is None:
                continue

            for IKData in ema.Skeleton.IKData:
                if IKData.Method == 0x00 and IKData.Flag0x00 == 0x02:
                    node0 = ema.Skeleton.Nodes[IKData.NodeIDs[0]]
                    node1 = ema.Skeleton.Nodes[IKData.NodeIDs[1]]
                    node2 = ema.Skeleton.Nodes[IKData.NodeIDs[2]]
                    node3 = ema.Skeleton.Nodes[IKData.NodeIDs[3]]
                    node4 = ema.Skeleton.Nodes[IKData.NodeIDs[4]]
                   
                    #def ProcessIKData0x00_02(arm, bone_names, ikflag0x01, node1_f, node2_f): 
                    result = ProcessIKData0x00_02(armature, [node0.Name,node1.Name,node2.Name,node3.Name,node4.Name], IKData.Flag0x01, node1.PreMatrixFloat, node2.PreMatrixFloat)
                    
                    ###start doing the hell maths
                    ##function to get a parent node_chain?
                    node1p_chain = CalculateNodeChain(ema.Skeleton, node1.Parent)
                    
                    ##Work through the chain and retrieve DAE-style local matrices, multiply as we go to retrieve world-space DAE matrix
                    ##TODO pre-calculate the chain and as much of the maths as possible so it doesn't get calculated every frame
                    ##(that should be part of the IK Chain class?)
                    matrix_world_dae = mathutils.Matrix.Translation(([0,0,0]))
                    j = len(node1p_chain)
                    while j > 0:
                        j -= 1
                        #local blender matrix
                        b = armature.pose.bones.get(node1p_chain[j].Name)
                        if b == None:
                            b = armature.pose.bones[0]
                        mat = b.matrix_basis
                        #parent
                        par = mathutils.Matrix.Translation(([0,0,0]))
                        if node1p_chain[j].Parent != -1:
                            par = ema.Skeleton.Nodes[node1p_chain[j].Parent].SBPMatrix.inverted()
                        #rest
                        rest = b.bone.matrix_local
                        #irestdae
                        irestdae = node1p_chain[j].SBPMatrix
                        
                        out = MatrixBlenderToDirectX(mat, rest, irestdae, par)
                        
                        matrix_world_dae = matrix_world_dae @ out

                    ##inverse DAE world matrix @ result to get local DAE result
                    #Not sure what is going on with all the transpositions, but it works!! Don't touch!
                    result0_local = (result[0].transposed() @ matrix_world_dae.inverted().transposed()).transposed()        
                    
                    ##function to return DAE result to blender format
                    mat = result0_local
                    par = mathutils.Matrix.Translation(([0,0,0]))
                    if node1.Parent != -1:
                        par = ema.Skeleton.Nodes[node1.Parent].SBPMatrix.inverted()
                    rest = armature.pose.bones[node1.Name].bone.matrix_local
                    irestdae = node1.SBPMatrix
                    
                    result0_blender = MatrixDirectXToBlender(mat, rest, irestdae, par)
                    
                    ## ASSIGN FINAL MATRIX TO THE POSEBONE
                    armature.pose.bones[node1.Name].matrix_basis = result0_blender
                    ## HOLY **** IT WORKED
                    
                    ##node2 is easy because the parent is node1, so we already have the parent world matrix
                    result1_local = result[0].inverted() @ result[1]
                    mat = result1_local
                    par = mathutils.Matrix.Translation(([0,0,0]))
                    if node2.Parent != -1:
                        par = ema.Skeleton.Nodes[node2.Parent].SBPMatrix.inverted()
                    rest = armature.pose.bones[node2.Name].bone.matrix_local
                    irestdae = node2.SBPMatrix
                    
                    result1_blender = MatrixDirectXToBlender(mat, rest, irestdae, par)
                    armature.pose.bones[node2.Name].matrix_basis = result1_blender
                    
                #elif IKData.Method == 0x01:
                    #node0 = ema.Skeleton.Nodes[IKData.NodeIDs[0]]
                    #node1 = ema.Skeleton.Nodes[IKData.NodeIDs[1]]
                    #node2 = ema.Skeleton.Nodes[IKData.NodeIDs[2]]
                    
                    #ProcessIKData0x01_00(arm, bone_names, ikfloats, ikflag0x01):
                    #result = ProcessIKData0x01_00(armature, [node0.Name,node1.Name,node2.Name],[IKData.Floats[0],IKData.Floats[1],IKData.Floats[2]],IKData.Flag0x01)
                    
                    #mat = result
                    #par = mathutils.Matrix.Translation(([0,0,0]))
                    #if node1.Parent != -1:
                    #    par = ema.Skeleton.Nodes[node1.Parent].SBPMatrix.inverted()
                    #rest = armature.pose.bones[node1.Name].bone.matrix_local
                    #irestdae = node1.SBPMatrix
                    
                    #result_blender = MatrixDirectXToBlender(mat, rest, irestdae, par)
                    
                    ## ASSIGN FINAL MATRIX TO THE POSEBONE
                    #armature.pose.bones[node1.Name].matrix_basis = result_blender
                    ## HOLY **** IT WORKED

def HermiteToBezier(p0, p1, t0, t1):
    b0 = p0
    b1 = p0 + t0/3
    b2 = p1 - t1/3
    b3 = p1
    
    return b0, b1, b2, b3

def HermiteToBezierSinglePoint(p0_x, p0_y, t0_x, t0_y):
    b0_x = p0_x
    b1_x = p0_x + t0_x/3
    
    b0_y = p0_y
    b1_y = p0_y + t0_y/3
    
    return b0_x, b0_y, b1_x, b1_y

def HermiteToBezierReverseSinglePoint(p0_x, p0_y, t0_x, t0_y):
    b0_x = p0_x
    b1_x = p0_x - t0_x/3
    
    b0_y = p0_y
    b1_y = p0_y - t0_y/3
    
    return b0_x, b0_y, b1_x, b1_y

class LoadAnimationData(bpy.types.Operator):
    """Load animation data from current .ema"""
    bl_idname = "usf4.load_animation_data"
    bl_label = "Load Animation Data"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):

        ema = None

        b_found = False
        for ad in armature_list:
            if bpy.context.object.data.name == ad.DatName:
                emo = ad.EMO
                ema = ad.EMA
                b_found = True
                break
        
        if b_found == False or ema == None or emo == None:
            print("Error - check ema & emo are loaded for this armature.")
            return {'CANCELLED'}
        
        armature = bpy.context.object
        if armature.animation_data == None:
            return {'CANCELLED'}
        elif armature.animation_data.action == None:
            return {'CANCELLED'}
        
        action = armature.animation_data.action
        bpy.context.scene.render.fps = 60
        
        for a in ema.Animations:
            if a.Name == action.name:
                #Clear curves ready for setup
                for f in action.fcurves:
                    action.fcurves.remove(f)
                
                for i in range(ema.Skeleton.NodeCount):
                    bone_name = ema.Skeleton.Nodes[i].Name
                    
                    #gather CMD tracks
                    local_tracks = []
                    for cmd in a.CMDTracks:
                        if cmd.BoneID == i:
                            armature.pose.bones[bone_name].animated = True
                            local_tracks.append(cmd)
                    
                    if len(local_tracks) > 0:
                        action.groups.new(bone_name)
                    
                    for cmd in local_tracks:
                        ttype = ""
                        if cmd.TransformType == 0:
                            ttype = "location"
                        elif cmd.TransformType == 1:
                            ttype = "rotation_euler"
                        else:
                            ttype = "scale"
                        
                        string = "pose.bones[\"" + bone_name + "\"]." + ttype
                        #Create new fcurve...
                        action.fcurves.new(string, index = (cmd.BitFlag & 0x03))
                        
                        action.fcurves[-1].mute = True
                        
                        #Populate keyframes...
                        j = 0
                        while j < cmd.StepCount:
                            value = a.ValueList[cmd.ValueIndicesList[j]]
                            value2 = 0
                            
                            if (j < cmd.StepCount -1):
                                value2 = a.ValueList[cmd.ValueIndicesList[j+1]]
                            
                            if cmd.TransformType == 1:
                                value = math.radians(value)
                                value2 = math.radians(value2)
                            action.fcurves[-1].keyframe_points.insert(cmd.StepsList[j], value)  
                            action.fcurves[-1].keyframe_points[-1].handle_left_type = 'ALIGNED'
                            action.fcurves[-1].keyframe_points[-1].handle_right_type = 'ALIGNED'
                            
                            #Check if we need to process tangents
                            if cmd.TangentIndicesList[j] != -1:
                                tangent_scalar = a.ValueList[cmd.TangentIndicesList[j]]
                                tangent_scalar2 = a.ValueList[cmd.TangentIndicesList[j+1]]
                                if cmd.TransformType == 1:
                                    tangent_scalar = math.radians(tangent_scalar)
                                    tangent_scalar2 = math.radians(tangent_scalar2)
                                #Get duration of step
                                step_length = cmd.StepsList[j+1] - cmd.StepsList[j]
                                t0_x = step_length
                                t0_y = tangent_scalar
                            
                                step_length_r = 0                        
                                #Get duration of "reverse" step
                                if j > 0:
                                    step_length_r = cmd.StepsList[j] - cmd.StepsList[j-1]
                                
                                b0_x,b0_y,b1_x,b1_y = HermiteToBezierSinglePoint(cmd.StepsList[j], value, t0_x, t0_y)
                                b2_x,b2_y,b3_x,b3_y = HermiteToBezierReverseSinglePoint(cmd.StepsList[j], value, step_length_r, t0_y)
                            
                                #def HermiteToBezierSinglePoint(p0_x, p0_y, t0_x, t0_y)
                                #return b0_x, b0_y, b1_x, b1_y
                            
                                action.fcurves[-1].keyframe_points[-1].handle_right = ((b1_x,b1_y))       
                               
                                if j > 0:
                                    action.fcurves[-1].keyframe_points[-1].handle_left = ((b3_x,b3_y))  
                                
                            else: 
                                action.fcurves[-1].keyframe_points[-1].handle_right = action.fcurves[-1].keyframe_points[-1].co
                                action.fcurves[-1].keyframe_points[-1].handle_left = action.fcurves[-1].keyframe_points[-1].co
                            
                            j += 1
                        
                        action.fcurves[-1].group = action.groups[bone_name]
                     
                    i += 1
                break
            armature.animation_data.action = action

        return {'FINISHED'}

def BuildValueList(cmd_list):
    #Just brute force the value table, inefficient but good enough for testing I hope
    values = []
    for c in cmd_list:
        #Force long Indices
        c.BitFlag = (c.BitFlag | 0x40)
        for i in range(c.StepCount):
            if i > 0 and i < c.StepCount -1 and c.TangentStorage[i] != None:
                c.IndicesList.append(0x40000000 | len(values))
                values.append(c.ValueStorage[i])
                values.append(c.TangentStorage[i]) 
            else:
                c.IndicesList.append(len(values))
                values.append(c.ValueStorage[i])                       
    
    return cmd_list, values


class SaveAnimationData(bpy.types.Operator, ExportHelper):
    """Save animation data to the current .ema"""
    bl_idname = "usf4.save_animation_data"
    bl_label = "Save Animation Data"
    bl_options = {'REGISTER'}
    filter_glob: StringProperty(
        default='*.ema',
        options={'HIDDEN'}
    )
    
    def get_enums(self, context):
        global armature_list
        
        items = [('New Animation','New Animation','New Animation')]
        
        ema = None
        armature = bpy.context.object
        
        for ad in armature_list:
            if armature.name == ad.ObjName:
                ema = ad.EMA
                break
        
        if ema is not None:
            for a in ema.Animations:
                items.append((a.Name,a.Name,a.Name))
        
        return items

    filename_ext = ".ema"
    
    # Custom properties
    append: EnumProperty(name="Target Animation",description="Animation to overwrite",items=get_enums,
        default=None)

    def invoke(self, context, event):
        #Fetch the ema from the armature_lust
        global armature_list
        ema = None
        armature = bpy.context.object
        
        for ad in armature_list:
            if armature.name == ad.ObjName:
                ema = ad.EMA
                break
        
        #Cancel if we don't find the armature/ema
        if ema is None:
            return {'CANCELLED'}
        
        #Otherwise, open the dialogue
        self.filepath = ema.Name
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}
    
    def execute(self, context):
        #Find the armature/ema *AGAIN* because of scope
        global armature_list
        
        ema = None
        armature = bpy.context.object
        
        for ad in armature_list:
            if armature.name == ad.ObjName:
                ema = ad.EMA
                break
        
        #Again, cancel if we don't find it, just in case
        if ema is None:
            print("Armature " + armature.name + " has no .ema loaded.")
            return {'CANCELLED'}

        if armature.animation_data == None:
            print("Armature " + armature.name + " has no animation data.")
            return {'CANCELLED'}
        
        ema_filepath = self.properties.filepath
        
        #Check the selected enum value, and find the animation to overwrite, or make a new animaton as appropriate
        b_new_animation = False
        ema_animation = None
        anim_index = -1
        if self.append == 'New Animation':
            b_new_animation = True
            ema_animation = Animation()
        else:
            for i in range(len(ema.Animations)):
                if ema.Animations[i].Name == self.append:
                    ema_animation = ema.Animations[i]
                    anim_index = i
                    break
        
        #Clear existing curves ready for new data
        action = armature.animation_data.action
            
        ema_animation.CMDTracks = []
        ema_animation.CMDTrackCount = 0
        ema_animation.ValueList = []
        ema_animation.ValueCount = 0
        ema_animation.CMDTrackPointerList = []
        
        #Divide fcurves by transform type
        temp_cmds = []
        for f in action.fcurves:
            temp_cmd = CMDTrack().FromFCurve(f, ema.Skeleton)
            
            bone_name = f.data_path[f.data_path.find("pose.bones[\"")+len("pose.bones[\""):f.data_path.find("\"].")]
            pbone = armature.pose.bones[bone_name]
            if temp_cmd.TransformType == 0 and pbone.absolute_translation == True:
                temp_cmd.BitFlag = (temp_cmd.BitFlag | 0x10)
            elif temp_cmd.TransformType == 1 and pbone.absolute_rotation == True:
                temp_cmd.BitFlag = (temp_cmd.BitFlag | 0x10)
            elif temp_cmd.TransformType == 2 and pbone.absolute_scale == True:
                temp_cmd.BitFlag = (temp_cmd.BitFlag | 0x10)
            
            temp_cmds.append(temp_cmd)
        
        temp_cmds = sorted(temp_cmds, key=attrgetter('TransformType','BoneID'))

        temp_cmds, temp_values = BuildValueList(temp_cmds)
        
        ema_animation.CMDTracks = temp_cmds
        ema_animation.ValueList = temp_values
        
        ema_animation.CMDTrackCount = len(temp_cmds)
        ema_animation.ValueCount = len(temp_values)
        
        ema_animation.Duration = bpy.context.scene.frame_end + 1
        
        if b_new_animation == False:
            ema.Animations[anim_index] = ema_animation
        else:
            ema_animation.Name = "NEW_ANIMATION"
            ema.Animations.append(ema_animation)
            ema.AnimationCount += 1
            ema.AnimationPointers.append(0)
        
        ema.Write(ema_filepath)
        
        return {'FINISHED'}

def pass_isbp_data(ema, emo):

    dict_EMAnodes = {}
    
    for i in range(len(ema.Skeleton.Nodes)):
        dict_EMAnodes.update({ema.Skeleton.Nodes[i].Name:i})

    i = 0
    for i in range(len(emo.Skeleton.Nodes)):
        emo_node = emo.Skeleton.Nodes[i]
        
        ema_match_id = dict_EMAnodes.get(emo_node.Name, -1)
        
        if ema_match_id == -1:
            i += 1
            continue
        
        ema.Skeleton.Nodes[ema_match_id].SBPMatrix = emo_node.SBPMatrix
     
    return ema   

class ImportEMO(bpy.types.Operator, ImportHelper):
    """Import additional data from an .emo"""
    bl_idname = "usf4.import_emo"
    bl_label = "Import EMO"
    bl_options = {'REGISTER', 'UNDO'}
    filter_glob: StringProperty(
        default='*.emo',
        options={'HIDDEN'}
    )
    
    def execute(self, context):
        global armature_list
        
        emo_filepath = self.properties.filepath
        
        ema = None
        
        with open(emo_filepath, "rb") as emo_file:
            emo = EMO(emo_file)
        
        #TESTING MULTIPLE ARMATURES        
        b_found = False
        for ad in armature_list:
            print(ad.ObjName,ad.DatName)
            if bpy.context.object.name == ad.ObjName:
                ad.EMO = emo
                ad.EMA = pass_isbp_data(ad.EMA, emo)
                b_found = True
                print(ad)
                break
        
        if b_found == False:
            print("Load EMA data first.")
            return {'CANCELLED'}
        
        #ema = pass_isbp_data(ema, emo)
        
        return {'FINISHED'}
            
class ImportEMA(bpy.types.Operator, ImportHelper):
    """Import animation data from an .ema"""
    bl_idname = "usf4.import_ema"
    bl_label = "Import EMA"
    bl_options = {'REGISTER', 'UNDO'}
    filter_glob: StringProperty(
        default='*.ema',
        options={'HIDDEN'}
    )
            
    def execute(self, context):    
        #Clear the emo so it's more obvious the data needs re-loading
        emo = None
        
        ema_filepath = self.properties.filepath
        armature = bpy.context.object
        
        global armature_list
        
        with open(ema_filepath, "rb") as ema_file:
            ema = EMA(ema_file)
        
        for n in ema.Skeleton.Nodes:
            #Load some extra data into the nodes to speed things up later
            #Don't do this, apparently this is very bad practice and probably causing the Undo errors/crashes
            #n.BlenderBone = armature.pose.bones.get(n.Name)
            n.NodeChain = CalculateNodeChain(ema.Skeleton, n.ID)
            
            #Nope
            #if n.BlenderBone == None and n.ID == 0:
            #    n.BlenderBone = armature.pose.bones[0]        
        
        ##TESTING MULTIPLE ARMATURES        
        b_found = False
        for ad in armature_list:
            if bpy.context.object.name == ad.ObjName:
                ad.EMO = None
                ad.EMA = ema
                b_found = True
                break
        
        if b_found == False:
            armature_list.append(USF4ArmatureData(bpy.context.object.name, bpy.context.object.data.name, ema))
        
        for ad in armature_list:
            print(ad.ObjName, ad.DatName)
        
        armature.animation_data_create()
        
        for a in ema.Animations:
            action = bpy.data.actions.get(a.Name)
            if action is None:
                action = bpy.data.actions.new(a.Name)
                action.use_fake_user = True

        return {'FINISHED'}

class EMAHandler(bpy.types.Panel):
    bl_label = 'EMA Handler'
    bl_idname = 'OBJECT_PT_main_panel'
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = 'object'
    
    @classmethod
    def poll(cls, context):
        return context.active_object is not None and context.active_object.type == 'ARMATURE'
    def draw(self, context):
        
        #global ema
        #global emo
        
        ema = None
        emo = None
        
        b_found = False
        for ad in armature_list:
            if bpy.context.object.name == ad.ObjName:
                emo = ad.EMO
                ema = ad.EMA
                b_found = True
                break
        
        layout = self.layout
        
        obj = context.object
        
        string = "No action"
        if obj.animation_data is not None:
            if obj.animation_data.action is not None:
                action = obj.animation_data.action
                string = action.name
            
        row = layout.row()
        
        if ema is not None:
            row.label(text="Loaded EMA: " + ema.Name)        
        else:
            row.label(text="Loaded EMA: None")
            
        row = layout.row()
        
        armature = bpy.data.armatures[context.object.name]

        row = layout.row()
        row.operator("usf4.import_ema", text="Load EMA...")
        
        row = layout.row()
        
        if emo is not None:
            row.label(text="Loaded EMO: " + emo.Name)
        else:
            row.label(text="Loaded EMO: None")
        
        row = layout.row()
        row.operator("usf4.import_emo", text="Load EMO...")
        
        if obj.animation_data is not None:    
            row = layout.row()
            row.prop_search(obj.animation_data, "action", bpy.data, "actions")
           
        row = layout.row()
             
        row.label(text="Active object is " + obj.name)
        
        row = layout.row()
        row.operator("usf4.load_animation_data", text="Load Animation Data")
        
        row = layout.row()
        row.operator("usf4.save_animation_data", text="Save Animation Data")
        
        row = layout.row()
        row.operator("usf4.hide_excess_bones", text="Hide Excess Bones")

        row = layout.row()
        row.operator("usf4.show_excess_bones", text="Show Excess Bones")

def quaternion_from_euler(roll_x, pitch_y, yaw_z):
    #Custom quaternion generator, needs moving to its own function and ideally
    #should be replaced with the mathutils version if I can figure out which sort of euler I need:
    dpitch = pitch_y
    dyaw = yaw_z
    droll = roll_x

    dSinPitch = math.sin(dpitch * 0.5)
    dCosPitch = math.cos(dpitch * 0.5)
    dSinYaw = math.sin(dyaw * 0.5)
    dCosYaw = math.cos(dyaw * 0.5)
    dSinRoll = math.sin(droll * 0.5)
    dCosRoll = math.cos(droll * 0.5)
    dCosPitchCosYaw = dCosPitch * dCosYaw
    dSinPitchSinYaw = dSinPitch * dSinYaw

    x = dSinRoll * dCosPitchCosYaw - dCosRoll * dSinPitchSinYaw
    y = dCosRoll * dSinPitch * dCosYaw + dSinRoll * dCosPitch * dSinYaw
    z = dCosRoll * dCosPitch * dSinYaw - dSinRoll * dSinPitch * dCosYaw
    w = dCosRoll * dCosPitchCosYaw + dSinRoll * dSinPitchSinYaw
        
    return mathutils.Quaternion((w,x,y,z))

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
 
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
 
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
 
    return roll_x, pitch_y, yaw_z # in radians

class USF4ArmatureData:
    def __init__ (self, objName, datName, load_ema = None, load_emo = None, load_fceema = None, last_action = None):
        self.ObjName = objName
        self.DatName = datName
        self.EMA = load_ema
        self.EMO = load_emo
        self.fceEMA = load_fceema
        self.last_action = last_action

class InsertUSF4Keyframe(bpy.types.Operator):
    """Insert USF4-style keyframe"""
    bl_idname = "usf4.insert_usf4_keyframe"
    bl_label = "Insert USF4 keyframe"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        global armature_list
        
        armature = bpy.context.active_object
        action = None
        if armature.animation_data is not None:
            action = armature.animation_data.action

        ema = None
        for ad in armature_list:
            if ad.ObjName == armature.name:
                ema = ad.EMA
                break

        #Gather selected posebones...
        for b in bpy.context.selected_pose_bones:
            #Fetch matching EMA bone...
            ema_bone = None
            for e in ema.Skeleton.Nodes:
                if e.Name == b.name:
                    ema_bone = e
                    print(ema_bone.Name)
                    break
            if ema_bone == None:
                print("Couldn't find selected bone in EMA skeleton.")
                return{'CANCELLED'}

            old_loc, old_rot, old_sca = b.matrix.decompose()
        
            #Do some hell maths
            par = mathutils.Matrix.Translation(([0,0,0]))
            if ema_bone.Parent != -1:
                par = ema.Skeleton.Nodes[ema_bone.Parent].SBPMatrix.inverted()         
            mat = b.matrix_basis
            irestdae = ema_bone.SBPMatrix
            rest = b.bone.matrix_local
            
            matrix_final = MatrixBlenderToDirectX(mat, rest, irestdae, par)
            
            if b.name == "LLegEff":
                matrix_final = b.matrix
            elif b.name == "RLegEff":
                matrix_final = b.matrix
            elif b.name == "LLegUp":
                matrix_final = b.matrix
            elif b.name == "RLegUp":
                matrix_final = b.matrix
            elif b.name == "LArmEff":
                matrix_final = b.matrix
            elif b.name == "LArmUp":
                matrix_final = b.matrix
            elif b.name == "RArmEff":
                matrix_final = b.matrix
            elif b.name == "RArmUp":
                matrix_final = b.matrix
                
            loc, rot, sca = matrix_final.decompose()
               
            #TODO something better than this that handles abs rotation
            #This is good enough to work with so long as TRS never gets rotated... I'm sure it's fine...
            #if b.absolute_translation == True:
                #loc = old_loc
            
            euler = euler_from_quaternion(rot.x,rot.y,rot.z,rot.w)
            
            #OK matrix_final SEEMS to be the correct local matrix in DirectX space
            #TODO handle IK control nodes/other absolute values

            #Fetch all curves associated with the bone
            fcurves = GetCurves(action,b.name)
            
            for f in fcurves:
                if f.data_path.find(".location") != -1:
                    f.keyframe_points.insert(bpy.context.scene.frame_current, loc[f.array_index])  
                    f.keyframe_points[-1].handle_left_type = 'ALIGNED'
                    f.keyframe_points[-1].handle_right_type = 'ALIGNED'
                elif f.data_path.find(".rotation_euler") != -1:
                    f.keyframe_points.insert(bpy.context.scene.frame_current, euler[f.array_index])  
                    f.keyframe_points[-1].handle_left_type = 'ALIGNED'
                    f.keyframe_points[-1].handle_right_type = 'ALIGNED'
                #Scaling REALLY seems to not work very well atm, disabled for now
                #elif f.data_path.find(".scale") != -1:
                    #f.keyframe_points.insert(bpy.context.scene.frame_current, sca[f.array_index])  
                    #f.keyframe_points[-1].handle_left_type = 'ALIGNED'
                    #f.keyframe_points[-1].handle_right_type = 'ALIGNED'
        
        #TODO see what the performance impact is if we "live" process IK while effectors are moving    
        IKProcessingHandler(bpy.context.scene)
            
        return{'FINISHED'}

#TODO Tidy this up, loads of duplicated code.
#Move code to a function for updating curves that takes inputs telling it if it's doing location/rotation/both
#Each operator then just calls the function with different inputs
class InsertUSF4KeyframeRotation(bpy.types.Operator):
    """Insert USF4-style rotation keyframe"""
    bl_idname = "usf4.insert_usf4_keyframe_rotation"
    bl_label = "Insert USF4 rotation keyframe"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        global ema
        global watchlist_ABS
        global watchlist_AO_blender
        
        #Fetch the action for the active armature...
        action = bpy.context.active_object.animation_data.action
        
        #Gather selected posebones...
        for b in bpy.context.selected_pose_bones:
            #Fetch matching EMA bone...
            ema_bone = None
            for e in ema.Skeleton.Nodes:
                if e.Name == b.name:
                    ema_bone = e
                    print(ema_bone.Name)
                    break
            if ema_bone == None:
                print("Couldn't find selected bone in EMA skeleton.")
                return{'CANCELLED'}

            old_loc, old_rot, old_sca = b.matrix.decompose()
        
            #Do some hell maths
            par = mathutils.Matrix.Translation(([0,0,0]))
            if ema_bone.Parent != -1:
                par = ema.Skeleton.Nodes[ema_bone.Parent].SBPMatrix.inverted()         
            mat = b.matrix_basis
            irestdae = ema_bone.SBPMatrix
            rest = b.bone.matrix_local
            
            matrix_final = MatrixBlenderToDirectX(mat, rest, irestdae, par)
            
            if b.name == "LLegEff":
                matrix_final = b.matrix
            elif b.name == "RLegEff":
                matrix_final = b.matrix
            elif b.name == "LLegUp":
                matrix_final = b.matrix
            elif b.name == "RLegUp":
                matrix_final = b.matrix
            elif b.name == "LArmEff":
                matrix_final = b.matrix
            elif b.name == "LArmUp":
                matrix_final = b.matrix
            elif b.name == "RArmEff":
                matrix_final = b.matrix
            elif b.name == "RArmUp":
                matrix_final = b.matrix
                
            loc, rot, sca = matrix_final.decompose()
               
            #TODO something better than this that handles abs rotation
            #This is good enough to work with so long as TRS never gets rotated... I'm sure it's fine...
            #if b.absolute_translation == True:
                #loc = old_loc
            
            euler = euler_from_quaternion(rot.x,rot.y,rot.z,rot.w)
            
            #OK matrix_final SEEMS to be the correct local matrix in DirectX space
            #TODO handle IK control nodes/other absolute values

            #Fetch all curves associated with the bone
            fcurves = GetCurves(action,b.name)
            
            for f in fcurves:
                if f.data_path.find(".rotation_euler") != -1:
                    f.keyframe_points.insert(bpy.context.scene.frame_current, loc[f.array_index])  
                    f.keyframe_points[-1].handle_left_type = 'ALIGNED'
                    f.keyframe_points[-1].handle_right_type = 'ALIGNED'
                #Scaling REALLY seems to not work very well atm, disabled for now
                #elif f.data_path.find(".scale") != -1:
                    #f.keyframe_points.insert(bpy.context.scene.frame_current, sca[f.array_index])  
                    #f.keyframe_points[-1].handle_left_type = 'ALIGNED'
                    #f.keyframe_points[-1].handle_right_type = 'ALIGNED'
        
        #TODO see what the performance impact is if we "live" process IK while effectors are moving    
        IKProcessingHandler(bpy.context.scene)
            
        return{'FINISHED'}
    
#TODO hmmm this might not work due to SRT order of operations? Eh, probably fine
class InsertUSF4KeyframeLocation(bpy.types.Operator):
    """Insert USF4-style location keyframe"""
    bl_idname = "usf4.insert_usf4_keyframe_location"
    bl_label = "Insert USF4 location keyframe"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        global ema
        global watchlist_ABS
        global watchlist_AO_blender
        
        #Fetch the action for the active armature...
        action = bpy.context.active_object.animation_data.action
        
        #Gather selected posebones...
        for b in bpy.context.selected_pose_bones:
            #Fetch matching EMA bone...
            ema_bone = None
            for e in ema.Skeleton.Nodes:
                if e.Name == b.name:
                    ema_bone = e
                    print(ema_bone.Name)
                    break
            if ema_bone == None:
                print("Couldn't find selected bone in EMA skeleton.")
                return{'CANCELLED'}

            old_loc, old_rot, old_sca = b.matrix.decompose()
        
            #Do some hell maths
            par = mathutils.Matrix.Translation(([0,0,0]))
            if ema_bone.Parent != -1:
                par = ema.Skeleton.Nodes[ema_bone.Parent].SBPMatrix.inverted()         
            mat = b.matrix_basis
            irestdae = ema_bone.SBPMatrix
            rest = b.bone.matrix_local
            
            matrix_final = MatrixBlenderToDirectX(mat, rest, irestdae, par)
            
            if b.name == "LLegEff":
                matrix_final = b.matrix
            elif b.name == "RLegEff":
                matrix_final = b.matrix
            elif b.name == "LLegUp":
                matrix_final = b.matrix
            elif b.name == "RLegUp":
                matrix_final = b.matrix
            elif b.name == "LArmEff":
                matrix_final = b.matrix
            elif b.name == "LArmUp":
                matrix_final = b.matrix
            elif b.name == "RArmEff":
                matrix_final = b.matrix
            elif b.name == "RArmUp":
                matrix_final = b.matrix
                
            loc, rot, sca = matrix_final.decompose()
               
            #TODO something better than this that handles abs rotation
            #This is good enough to work with so long as TRS never gets rotated... I'm sure it's fine...
            #if b.absolute_translation == True:
                #loc = old_loc
            #NARRATOR: It was not fine
            
            euler = euler_from_quaternion(rot.x,rot.y,rot.z,rot.w)
            
            #OK matrix_final SEEMS to be the correct local matrix in DirectX space
            #TODO handle IK control nodes/other absolute values

            #Fetch all curves associated with the bone
            fcurves = GetCurves(action,b.name)
            
            for f in fcurves:
                if f.data_path.find(".location") != -1:
                    f.keyframe_points.insert(bpy.context.scene.frame_current, euler[f.array_index])  
                    f.keyframe_points[-1].handle_left_type = 'ALIGNED'
                    f.keyframe_points[-1].handle_right_type = 'ALIGNED'
                #Scaling REALLY seems to not work very well atm, disabled for now
                #elif f.data_path.find(".scale") != -1:
                    #f.keyframe_points.insert(bpy.context.scene.frame_current, sca[f.array_index])  
                    #f.keyframe_points[-1].handle_left_type = 'ALIGNED'
                    #f.keyframe_points[-1].handle_right_type = 'ALIGNED'
        
        #TODO see what the performance impact is if we "live" process IK while effectors are moving    
        IKProcessingHandler(bpy.context.scene)
        
        return{'FINISHED'}

class HideExcessBones(bpy.types.Operator):
    """Hide excess bones"""
    bl_idname = "usf4.hide_excess_bones"
    bl_label = "Hide excess bones"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        global armature_list
        
        for ad in armature_list:
            if bpy.context.object.name == ad.ObjName:
                for n in ad.EMA.Skeleton.Nodes:
                    if n.BitFlag == 0 and bpy.context.object.pose.bones.get(n.Name) is not None:
                        bpy.context.object.pose.bones.get(n.Name).bone.hide = True
        
        return{'FINISHED'}

class ShowExcessBones(bpy.types.Operator):
    """Show excess bones"""
    bl_idname = "usf4.show_excess_bones"
    bl_label = "Show excess bones"
    bl_options = {'REGISTER', 'UNDO'}
    
    def execute(self, context):
        global armature_list
        
        for ad in armature_list:
            if bpy.context.object.name == ad.ObjName:
                for n in ad.EMA.Skeleton.Nodes:
                    if n.BitFlag == 0 and bpy.context.object.pose.bones.get(n.Name) is not None:
                        bpy.context.object.pose.bones.get(n.Name).bone.hide = False
        
        return{'FINISHED'}

class VIEW3D_MT_insert_keyframe_usf4(bpy.types.Menu):
    bl_label = "Insert Keyframe USF4"
    bl_idname = "VIEW3D_MT_insert_keyframe_usf4"

    def draw(self, context):
        layout = self.layout
        layout.operator("usf4.insert_usf4_keyframe")
        #row = layout.row()
        #row.operator("usf4.insert_usf4_keyframe_rotation")
        #row = layout.row()
        #row.operator("usf4.insert_usf4_keyframe_location")

addon_keymaps = []

@persistent
def ActionWatcher(self, context):
    global armature_list
    
    for ad in armature_list:
        armature = bpy.data.objects.get(ad.ObjName)
        
        if armature is not None and armature.animation_data is not None:
            action = armature.animation_data.action
            
            #Check to see if the armature hasn't had an action before
            #or, if it has an action but it's changed since last check
            if action is not None and ad.last_action is None:
                ad.last_action = action.name
                update_action(ad.EMA, armature)
            elif action is not None and action.name != ad.last_action:
                ad.last_action = action.name
                update_action(ad.EMA, armature)

def update_action(ema, armature):
    action = armature.animation_data.action
    
    #Clear flags
    for b in armature.pose.bones:
        b.absolute_translation = False
        b.absolute_rotation = False
        b.absolute_scale = False
        b.animated = False
    
    #Find the animation so we can retrieve the CMDTracks
    Animation = None
    for anim in ema.Animations:
        if anim.Name == action.name:
            Animation = anim
            break
       
    #Set flags
    if Animation is not None:
        bpy.context.scene.render.fps = 60
        bpy.context.scene.frame_start = 0
        bpy.context.scene.frame_end = Animation.Duration - 1
        for cmd in Animation.CMDTracks:
            bone_name = ema.Skeleton.Nodes[cmd.BoneID].Name
            b = armature.pose.bones.get(bone_name)
            if b is not None:
                b.animated = True
                if (cmd.BitFlag & 0x10) == 0x10:
                    if cmd.TransformType == 0:
                        b.absolute_translation = True
                    elif cmd.TransformType == 1:
                        b.absolute_rotation = True
                    else:
                        b.absolute_scale = True   

def register():
    bpy.utils.register_class(EMAHandler)
    bpy.utils.register_class(ImportEMA)
    bpy.utils.register_class(ImportEMO)
    bpy.utils.register_class(LoadAnimationData)
    bpy.utils.register_class(SaveAnimationData)
    bpy.utils.register_class(HideExcessBones)
    bpy.utils.register_class(ShowExcessBones)
    bpy.utils.register_class(InsertUSF4Keyframe)
    #bpy.utils.register_class(InsertUSF4KeyframeRotation)
    #bpy.utils.register_class(InsertUSF4KeyframeLocation)
    bpy.utils.register_class(VIEW3D_MT_insert_keyframe_usf4)
    
    #Setup keyboard shortcuts for custom keyframing menu
    wm = bpy.context.window_manager
    kc = wm.keyconfigs.addon   
    if kc:
        km = wm.keyconfigs.addon.keymaps.new(name='3D View', space_type='VIEW_3D')
        kmi = km.keymap_items.new('wm.call_menu', 'U', 'PRESS', ctrl=False, shift=False, alt=False)
        kmi.properties.name =  VIEW3D_MT_insert_keyframe_usf4.bl_idname
        addon_keymaps.append((km, kmi))
    
    bpy.app.handlers.depsgraph_update_pre.append(ActionWatcher)
    bpy.app.handlers.frame_change_post.append(EMAProcessing)
    bpy.app.handlers.frame_change_post.append(IKProcessingHandler)

def unregister():
    bpy.utils.unregister_class(EMAHandler)               
    bpy.utils.unregister_class(ImportEMA)     
    bpy.utils.unregister_class(ImportEMO)
    bpy.utils.unregister_class(LoadAnimationData)
    bpy.utils.unregister_class(SaveAnimationData)    
    bpy.utils.unregister_class(HideExcessBones)    
    bpy.utils.unregister_class(ShowExcessBones)    
    bpy.utils.unregister_class(InsertUSF4Keyframe)   
    #bpy.utils.unregister_class(InsertUSF4KeyframeRotation)    
    #bpy.utils.unregister_class(InsertUSF4KeyframeLocation)    
    bpy.utils.unregister_class(VIEW3D_MT_insert_keyframe_usf4)   
    for km, kmi in addon_keymaps:
        km.keymap_items.remove(kmi)
    addon_keymaps.clear()

    for h in bpy.app.handlers.depsgraph_update_pre:
        if h.__name__ == 'ActionWatcher':
            bpy.app.handlers.depsgraph_update_pre.remove(h)
   
    for h in bpy.app.handlers.frame_change_post:
        if h.__name__ == 'EMAProcessing':
            bpy.app.handlers.frame_change_post.remove(h)

    for h in bpy.app.handlers.frame_change_post:
        if h.__name__ == 'IKProcessingHandler':
            bpy.app.handlers.frame_change_post.remove(h)

# This allows you to run the script directly from Blender's Text editor
# to test the add-on without having to install it.
if __name__ == "__main__":
    register()
