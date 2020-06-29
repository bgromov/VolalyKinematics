//
//  PointinigModel.swift
//  Pointer
//
//  Created by Boris Gromov on 26/05/2019.
//  Copyright © 2019 Volaly. All rights reserved.
//

import Foundation
import simd

import Transform

public class PointingModel: SurfaceDelegate {
    public enum Handedness {
        case ignore
        case leftHand
        case rightHand
    }

    private(set) public var pointWith: Handedness {
        didSet {
            self.updateKinematics()
        }
    }

    // Human kinematic parameters
    private(set) public var bodyHeight: Double! {
        didSet {
            self.kinematicsFrom(bodyHeight: self.bodyHeight)
        }
    }

    private(set) var shoulderHeight: Double!
    private(set) var shoulderToNeck: Double!
    private(set) var shoulderToEyes: Double!
    private(set) var shoulderToWrist: Double!
    private(set) var wristToFinger: Double!

    // Transformations
    private(set) public var footprintToNeckTf: Transform!
    private(set) public var neckToEyesTf: Transform!
    private(set) public var neckToShoulderTf: Transform!
    private(set) public var shoulderToWristTf: Transform!
    private(set) public var wristToFingerTf: Transform!

    private var worldTf: Transform
    private var imuTf: Transform

    private var rayTf: Transform?
    private var pointerTf: Transform?
    private var fingerTf: Transform?

    private(set) public var surface: Surface

    public var ray: Transform? {
        return rayTf
    }
    public var pointer: Transform? {
        return pointerTf
    }
    public var finger: Transform? {
        return fingerTf
    }

    private func kinematicsFrom(bodyHeight: Double) {
        // bodyHeight: 1.835 // 1.67

        let scaleFactor: Double = bodyHeight / 1.835 // 0.91008174
        shoulderHeight  = 1.47 * scaleFactor // 1.34
        shoulderToNeck  = 0.18 * scaleFactor // 0.16
        shoulderToEyes  = 0.22 * scaleFactor // 0.17
        shoulderToWrist = 0.51 * scaleFactor // 0.45
        wristToFinger   = 0.18 * scaleFactor // 0.16

        self.updateKinematics()
    }

    private func updateKinematics() {
        footprintToNeckTf = Transform(simd_double3x3(1.0),
                                      simd_double3(x: 0, y: 0, z: shoulderHeight))
        neckToEyesTf      = Transform(simd_double3x3(1.0),
                                      simd_double3(x: 0, y: 0, z: shoulderToEyes))

        let sign: Double = (self.pointWith == .rightHand) ? -1.0 :
                           (self.pointWith == .leftHand)  ?  1.0 : 0.0

        neckToShoulderTf  = Transform(imuTf.rotation,
                                      simd_double3(x: 0, y: sign * shoulderToNeck, z: 0))

        shoulderToWristTf = Transform(simd_double3x3(1.0),
                                      simd_double3(x: shoulderToWrist, y: 0, z: 0))
        wristToFingerTf   = Transform(simd_double3x3(1.0),
                                      simd_double3(x: wristToFinger, y: 0, z: 0))
    }


    /// Public methods

    public init(bodyHeight: Double, surface: Surface, pointWith: Handedness = .ignore) {
        self.surface = surface

        imuTf = Transform.identity
        worldTf = Transform.identity
        self.pointWith = pointWith

        self.surface.delegate = self

        // Trick to trigger didSet() of bodyHeight
        defer {
            self.bodyHeight = bodyHeight
        }
    }

    public func setWorldTransform(_ tf: Transform) {
        worldTf = tf
        updateModel()
    }

    public func setImuTransform(_ tf: Transform) {
        imuTf = tf
        updateModel()
    }

    public func surface(didUpdateParam param: String, with value: simd_double3?) {
        self.updateModel()
    }

    public func updateModel() {
        self.updateKinematics()

        let fingerTf = footprintToNeckTf * neckToShoulderTf * shoulderToWristTf * wristToFingerTf
        let eyesTf = footprintToNeckTf * neckToEyesTf

        let ray_vec = simd_normalize(fingerTf.origin - eyesTf.origin)
        let ray_yaw_rot = simd_quatd(roll: 0.0, pitch: 0.0, yaw: atan2(ray_vec.y, ray_vec.x))
        let ray_new_x = simd_double3x3(ray_yaw_rot) * simd_double3(x: 1.0, y: 0.0, z: 0.0)
        let ray_pitch_rot = simd_quatd(roll: 0.0,
                                       pitch: atan2(-ray_vec.z, simd_dot(ray_vec, ray_new_x)),
                                       yaw: 0.0)

        self.rayTf = Transform(ray_yaw_rot * ray_pitch_rot, eyesTf.origin)
        self.pointerTf = surface.intersectWith(ray: rayTf!)
        self.fingerTf = fingerTf
    }
}
