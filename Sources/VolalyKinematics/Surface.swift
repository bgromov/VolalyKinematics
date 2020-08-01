//
//  Surface.swift
//  VolalyKinematics
//
//  Created by Boris Gromov on 01/08/2019.
//  Copyright © 2019 Volaly. All rights reserved.
//

import Foundation
import simd

import Transform

@dynamicCallable
public class SurfaceParams {
    var kv: KeyValuePairs<String, simd_double3?>
    var dict: [String: simd_double3?]

    public init(_ params: KeyValuePairs<String, simd_double3?>) {
        self.kv = params
        self.dict = Dictionary(uniqueKeysWithValues: Array(params))
    }
    func dynamicallyCall(withArguments args: [simd_double3?]) {
        assert(args.count == dict.count, "Number of unnamed arguments does not match number of initialized arguments")
        for (i, newVal) in args.enumerated() {
            let (key, _) = kv[i]
            guard let oldVal = dict[key] else {
                assert(false, "This should never happen")
                return
            }
            dict[key] = newVal ?? oldVal
        }
    }
    func dynamicallyCall(withKeywordArguments args: KeyValuePairs<String, simd_double3?>) {
        for arg in args {
            guard let val = dict[arg.key] else {
                assert(false, "Property '\(arg.key)' does not exist")
                return
            }
            dict[arg.key] = arg.value ?? val
        }
    }
    subscript(key: String) -> simd_double3?? {
        get {
            return dict[key]
        }
        set(newVal) {
            guard let _ = dict[key] else {
                assert(false, "Key '\(key)' does not exist")
                return
            }
            dict[key] = newVal
        }
    }
}

public protocol SurfaceDelegate: class {
    func surface(didUpdateParam param: String, with value: simd_double3?)
}

@dynamicMemberLookup
public protocol Surface {
    var delegate: SurfaceDelegate? {get set}

    var params: SurfaceParams {get}
    func intersectWith(ray tf: Transform) -> Transform?
    subscript(dynamicMember member: String) -> simd_double3? {get}
}

public extension Surface {
    // Default implementation
    subscript(dynamicMember member: String) -> simd_double3? {
        get {
            guard let param = params[member] else {
                assert(false, "Property '\(member)' does not exist")
                return nil
            }
            return param
        }
        set (newValue) {
            guard let _ = params[member] else {
                assert(false, "Property '\(member)' does not exist")
                return
            }
            params[member] = newValue
            self.delegate?.surface(didUpdateParam: member, with: newValue)
        }
    }
}

public class Plane: Surface {
    public weak var delegate: SurfaceDelegate?
    public var params: SurfaceParams

    public init(normal: simd_double3, point: simd_double3) {
        self.params = SurfaceParams(["normal": simd_normalize(normal),
                                     "point" : point])
    }

    public func intersectWith(ray: Transform) -> Transform? {
        guard let normal = self.normal else {
            assert(false, "Normal is not set")
            return nil
        }
        guard let point = self.point else {
            assert(false, "Point is not set")
            return nil
        }

        let u = ray.origin - ray * simd_double3(x: 1.0, y: 0.0, z: 0.0)
        let w = ray.origin - point
        let n = normal

        let D = simd_dot(n, u)
        let N = -simd_dot(n, w)

        if fabs(D) > Double.ulpOfOne {
            let sI = N / D
            if sI >= 0.0 {
                return nil
            }

            let p = ray.origin + sI * u
            let yaw = atan2(p.y, p.x)
            let q = simd_quatd(roll: 0.0, pitch: 0.0, yaw: yaw)

            return Transform(q, p)
        }

        return nil
    }
}

public class HorizontalPlane: Plane {
    private override init(normal: simd_double3, point: simd_double3) {
        super.init(normal: normal, point: point)
    }

    public init(point: simd_double3) {
        let normal = simd_double3(x: 0.0, y: 0.0, z: 1.0)
        super.init(normal: normal, point: point)
    }
}

public class Cylinder: Surface {
    public weak var delegate: SurfaceDelegate?
    public var params: SurfaceParams

    public init(axis: simd_double3, point: simd_double3) {
        self.params = SurfaceParams(["axis" : simd_normalize(axis),
                                     "point": point])
    }

    public func intersectWith(ray: Transform) -> Transform? {
        guard let axis = self.axis else {
            assert(false, "Axis is not set")
            return nil
        }
        guard let point = self.point else {
            assert(false, "Point is not set")
            return nil
        }

        let u = ray.origin - ray * simd_double3(x: 1.0, y: 0.0, z: 0.0)
        let n = axis

        let ang = simd_norm_one(simd_cross(u, n))

        // Check if ray is parallel to cylinder axis
        if ang > Double.ulpOfOne {
            // Project point on axis and make a vector along it with resulting length
            let l = simd_dot(point, n) * n
            // Calculate radius vector as a difference of l and point
            let r = simd_norm_one(point - l)

            // If it is a vertical cylinder the difference must be zero
            //let r2d = simd_norm_one(simd_double2(x: point.x, y: point.y))
            //print("Diff: \(r2d - r)")

            let (_, pitch, yaw) = ray.rotation.rpy

            // FIXME: this will only work in case of vertical cylinder
            let x = r * cos(yaw)
            let y = r * sin(yaw)
            let z = -r * tan(pitch)

            // Point on the cylinder
            let p = ray.origin + simd_double3(x: x, y: y, z: z)

            // Turn in the direction of pointing in horizontal plane
            let q = simd_quatd(roll: 0.0, pitch: 0.0, yaw: yaw)

            return Transform(q, p)
        }

        return nil
    }
}

public class Sphere: Surface {
    public weak var delegate: SurfaceDelegate?
    public var params: SurfaceParams

    public init(point: simd_double3) {
        self.params = SurfaceParams(["point": point])
    }

    public func intersectWith(ray: Transform) -> Transform? {
        guard let point = self.point else {
            assert(false, "Point is not set")
            return nil
        }

        // Sphere radius
        let r = simd_norm_one(point)
        // Point on sphere
        let p = ray * simd_double3(x: r, y: 0.0, z: 0.0)

        // Get heading
        let (_, _, yaw) = ray.rotation.rpy

        // Turn in the direction of pointing in horizontal plane
        let q = simd_quatd(roll: 0.0, pitch: 0.0, yaw: yaw)

        return Transform(q, p)
    }
}
