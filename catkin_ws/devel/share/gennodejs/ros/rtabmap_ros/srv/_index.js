
"use strict";

let GetMap = require('./GetMap.js')
let GlobalBundleAdjustment = require('./GlobalBundleAdjustment.js')
let SetLabel = require('./SetLabel.js')
let DetectMoreLoopClosures = require('./DetectMoreLoopClosures.js')
let ListLabels = require('./ListLabels.js')
let PublishMap = require('./PublishMap.js')
let GetMap2 = require('./GetMap2.js')
let ResetPose = require('./ResetPose.js')
let GetNodesInRadius = require('./GetNodesInRadius.js')
let GetNodeData = require('./GetNodeData.js')
let GetPlan = require('./GetPlan.js')
let RemoveLabel = require('./RemoveLabel.js')
let AddLink = require('./AddLink.js')
let SetGoal = require('./SetGoal.js')
let LoadDatabase = require('./LoadDatabase.js')
let CleanupLocalGrids = require('./CleanupLocalGrids.js')

module.exports = {
  GetMap: GetMap,
  GlobalBundleAdjustment: GlobalBundleAdjustment,
  SetLabel: SetLabel,
  DetectMoreLoopClosures: DetectMoreLoopClosures,
  ListLabels: ListLabels,
  PublishMap: PublishMap,
  GetMap2: GetMap2,
  ResetPose: ResetPose,
  GetNodesInRadius: GetNodesInRadius,
  GetNodeData: GetNodeData,
  GetPlan: GetPlan,
  RemoveLabel: RemoveLabel,
  AddLink: AddLink,
  SetGoal: SetGoal,
  LoadDatabase: LoadDatabase,
  CleanupLocalGrids: CleanupLocalGrids,
};
