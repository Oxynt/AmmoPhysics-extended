async function AmmoPhysics() {

    if ('Ammo' in window === false) {

        console.error('AmmoPhysics: Couldn\'t find Ammo.js');
        return;

    }

    //-------------------------------------------------------------------
    // Init ammo
    //-------------------------------------------------------------------
    const AmmoLib = await Ammo();

	
	//common function
	const transform = new AmmoLib.btTransform();
	
	const vectGravity = new AmmoLib.btVector3(0, -9.8, 0);
	const vectVelocity = new AmmoLib.btVector3();
	const vectAngular = new AmmoLib.btVector3();

    const frameRate = 60;
    const collisionConfiguration = new AmmoLib.btDefaultCollisionConfiguration();
    const dispatcher = new AmmoLib.btCollisionDispatcher(collisionConfiguration);
    const broadphase = new AmmoLib.btDbvtBroadphase();
    const solver = new AmmoLib.btSequentialImpulseConstraintSolver();
    const world = new AmmoLib.btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	
	world.setGravity(vectGravity);
	
    //-------------------------------------------------------------------
    // create ammo primitives from three geometry
    //-------------------------------------------------------------------
    function getShape(geometry) {

        const parameters = geometry.parameters;

        if (geometry.type === 'BoxGeometry') {

            const sx = parameters.width !== undefined ? parameters.width / 2 : 0.5;
            const sy = parameters.height !== undefined ? parameters.height / 2 : 0.5;
            const sz = parameters.depth !== undefined ? parameters.depth / 2 : 0.5;

            const shape = new AmmoLib.btBoxShape(new AmmoLib.btVector3(sx, sy, sz));
            shape.setMargin(0.05);

            return shape;

        } else if (geometry.type === 'PlaneGeometry') {

            const sx = parameters.width !== undefined ? parameters.width / 2 : 0.5;
            const sy = parameters.height !== undefined ? parameters.height / 2 : 0.5;

            const shape = new AmmoLib.btBoxShape(new AmmoLib.btVector3(sx, sy, 0.01));
            shape.setMargin(0.05);

            return shape;

        } else if (geometry.type === 'SphereGeometry') {

            const sr = parameters.radius !== undefined ? parameters.radius : 1;

            const shape = new AmmoLib.btSphereShape(sr);
            shape.setMargin(0);

            return shape;

        } else if (geometry.type === 'CylinderGeometry' && parameters.radiusTop === parameters.radiusBottom) {

            const st = parameters.radiusTop !== undefined ? parameters.radiusTop : 0.5;
            const sy = parameters.height !== undefined ? parameters.height : 1;

            const shape = new AmmoLib.btCylinderShape(new AmmoLib.btVector3(st, sy * 0.5, st));
            shape.setMargin(0.05);

            return shape;

        } else if (geometry.type === 'ConeGeometry') {

            const sr = parameters.radius !== undefined ? parameters.radius : 1;
            const sy = parameters.height !== undefined ? parameters.height : 1;

            const shape = new AmmoLib.btConeShape(sr, sy);
            shape.setMargin(0);

            return shape;
			
		} else if (geometry.name === 'hull') {
			
            const shape = new AmmoLib.btConvexHullShape();

            //new ammo triangles
            let triangle, triangle_mesh = new Ammo.btTriangleMesh;

            //declare triangles position vectors
            let vectA = new AmmoLib.btVector3(0, 0, 0);
            let vectB = new AmmoLib.btVector3(0, 0, 0);
            let vectC = new AmmoLib.btVector3(0, 0, 0);

            //retrieve vertices positions from object
            let verticesPos = geometry.getAttribute('position').array;
            let triangles = [];
            for (let i = 0; i < verticesPos.length; i += 3) {
                triangles.push({
                    x: verticesPos[i],
                    y: verticesPos[i + 1],
                    z: verticesPos[i + 2]
                })
            }

            //use triangles data to draw ammo shape
            for (let i = 0; i < triangles.length - 3; i += 3) {

                vectA.setX(triangles[i].x);
                vectA.setY(triangles[i].y);
                vectA.setZ(triangles[i].z);
                shape.addPoint(vectA, true);

                vectB.setX(triangles[i + 1].x);
                vectB.setY(triangles[i + 1].y);
                vectB.setZ(triangles[i + 1].z);
                shape.addPoint(vectB, true);

                vectC.setX(triangles[i + 2].x);
                vectC.setY(triangles[i + 2].y);
                vectC.setZ(triangles[i + 2].z);
                shape.addPoint(vectC, true);

                triangle_mesh.addTriangle(vectA, vectB, vectC, true);
            }
			
			AmmoLib.destroy(vectA);
            AmmoLib.destroy(vectB);
            AmmoLib.destroy(vectC);

            shape.setMargin(0);

            return shape
			
		} else if (geometry.name === 'triangle') {
		
            //new ammo triangles
            let triangle, triangle_mesh = new AmmoLib.btTriangleMesh();

            //declare triangles position vectors
            let vectA = new AmmoLib.btVector3(0, 0, 0);
            let vectB = new AmmoLib.btVector3(0, 0, 0);
            let vectC = new AmmoLib.btVector3(0, 0, 0);

            //retrieve vertices positions from object
            let verticesPos = geometry.getAttribute('position').array;
            let triangles = [];
            for (let i = 0; i < verticesPos.length; i += 3) {
                triangles.push({
                    x: verticesPos[i],
                    y: verticesPos[i + 1],
                    z: verticesPos[i + 2]
                })
            }

            //use triangles data to draw ammo shape
            for (let i = 0; i < triangles.length - 3; i += 3) {

                vectA.setX(triangles[i].x);
                vectA.setY(triangles[i].y);
                vectA.setZ(triangles[i].z);

                vectB.setX(triangles[i + 1].x);
                vectB.setY(triangles[i + 1].y);
                vectB.setZ(triangles[i + 1].z);

                vectC.setX(triangles[i + 2].x);
                vectC.setY(triangles[i + 2].y);
                vectC.setZ(triangles[i + 2].z);

                triangle_mesh.addTriangle(vectA, vectB, vectC, true);
            }
			
			AmmoLib.destroy(vectA);
            AmmoLib.destroy(vectB);
            AmmoLib.destroy(vectC);

			//var shape = new AmmoLib.btBvhTriangleMeshShape(triangle_mesh, true);
			
			var shape = new Ammo.btConvexTriangleMeshShape( triangle_mesh, true);
			
			shape.setMargin(0);
			
			return shape

		}

    }

    const meshes = [];
    const meshMap = new WeakMap();


    //-------------------------------------------------------------------
    //THREE to Ammo parsing function
    //-------------------------------------------------------------------
    function addMesh(mesh, body, mass) {

        let shape;

        if (body.isGroup) { //convert THREE.goup to Compound Shape
            shape = getCompoundShape(body);
        } else { //convert THREE.geometry to Shape
            shape = getShape(body.geometry);
        }

        if (shape !== null) {

            if (mesh.isInstancedMesh) {

                handleInstancedMesh(mesh, mass, shape);

            } else {

                handleMesh(mesh, mass, shape);

            }
        }
    }

    function getCompoundShape(body) {

        const shapes = [];
        const positions = [];
        const quaternions = [];

        body.traverse(function(child) {
            if (child.isMesh) {

                shapes.push(getShape(child.geometry));
                positions.push(child.position);
                quaternions.push(child.quaternion);
            }
        });

        const compShape = new AmmoLib.btCompoundShape();

        for (let i = 0; i < shapes.length; i++) {

            shapes[i].setMargin(0);
            let transform = meshTransform(positions[i], quaternions[i]);
            compShape.addChildShape(transform, shapes[i]);
        }

        return compShape;
    }

    //-------------------------------------------------------------------
    //Init mesh shapes
    //-------------------------------------------------------------------
    function handleMesh(mesh, mass, shape) {

        const position = mesh.position;
        const quaternion = mesh.quaternion;

        const transform = meshTransform(position, quaternion);

        const motionState = new AmmoLib.btDefaultMotionState(transform);

        const localInertia = new AmmoLib.btVector3(0, 0, 0);
        shape.calculateLocalInertia(mass, localInertia);

        const rbInfo = new AmmoLib.btRigidBodyConstructionInfo(mass, motionState, shape, localInertia);

        const body = new AmmoLib.btRigidBody(rbInfo);

        body.index = mesh.index;
        body.setSleepingThresholds(0.0, 0.1);
        //body.setFriction( 4 );

        world.addRigidBody(body);

        meshes.push(mesh);
        meshMap.set(mesh, body);

    }

    function meshTransform(position, quaternion) {

        const transform = new AmmoLib.btTransform();
        transform.setIdentity();
        transform.setOrigin(new AmmoLib.btVector3(position.x, position.y, position.z));
        transform.setRotation(new AmmoLib.btQuaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w));

        return transform;
    }

    //-------------------------------------------------------------------
    //Init Instanced mesh shapes
    //-------------------------------------------------------------------
    function handleInstancedMesh(mesh, mass, shape) {

        const array = mesh.instanceMatrix.array;

        const bodies = [];

        for (let i = 0; i < mesh.count; i++) {

            const indexes = i * 16;

            const transform = new AmmoLib.btTransform();
            transform.setFromOpenGLMatrix(array.slice(indexes, indexes + 16));

            const motionState = new AmmoLib.btDefaultMotionState(transform);

            const localInertia = new AmmoLib.btVector3(0, 0, 0);
            shape.calculateLocalInertia(mass, localInertia);

            const rbInfo = new AmmoLib.btRigidBodyConstructionInfo(mass, motionState, shape, localInertia);

            const body = new AmmoLib.btRigidBody(rbInfo);

            world.addRigidBody(body);

            bodies.push(body);

        }

        mesh.instanceMatrix.setUsage(35048); // THREE.DynamicDrawUsage = 35048
        meshes.push(mesh);

        meshMap.set(mesh, bodies);
    }

    //-------------------------------------------------------------------
    //set new position
    //-------------------------------------------------------------------
    function setMeshPosition(mesh, position, quaternion, index) {

        if (mesh.isInstancedMesh) {

            const bodies = meshMap.get(mesh);
            const body = bodies[index];

            transform.setIdentity();
            transform.setOrigin(new AmmoLib.btVector3(position.x, position.y, position.z));

            transform.setRotation(new AmmoLib.btQuaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w));
            body.setWorldTransform(transform);

        } else {

            const body = meshMap.get(mesh);

            transform.setIdentity();
            transform.setOrigin(new AmmoLib.btVector3(position.x, position.y, position.z));

            transform.setRotation(new AmmoLib.btQuaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w));
            body.setWorldTransform(transform);

        }
    }

    //-------------------------------------------------------------------
    //set new velocity and rotational velocity
    //-------------------------------------------------------------------
    function setMeshVelocity(mesh, velocity, angular, index) {
		

        if (mesh.isInstancedMesh) {

            const bodies = meshMap.get(mesh);
            const body = bodies[index];
			
			body.activate();

			vectAngular.setValue(angular.x, angular.y, angular.z);
            body.setAngularVelocity(vectAngular);

			vectVelocity.setValue(velocity.x, velocity.y, velocity.z);
            body.setLinearVelocity(vectVelocity);

        } else {

            const body = meshMap.get(mesh);
            
			vectAngular.setValue(angular.x, angular.y, angular.z);
            body.setAngularVelocity(vectAngular);

			vectVelocity.setValue(velocity.x, velocity.y, velocity.z);
            body.setLinearVelocity(vectVelocity);

			body.activate();
        }

    }

    //-------------------------------------------------------------------
    //Init contacts
    //-------------------------------------------------------------------

    let cbContactResult;

    function setupContact(bodyA, indexB, contact) {

        cbContactResult = new AmmoLib.ConcreteContactResultCallback();

        cbContactResult.addSingleResult = function(cp, colObj0Wrap, partId0, index0, colObj1Wrap, partId1, index1) {

            let contactPoint = Ammo.wrapPointer(cp, Ammo.btManifoldPoint);

            const distance = contactPoint.getDistance();

            if (distance > 0) return;

            let colWrapper0 = Ammo.wrapPointer(colObj0Wrap, Ammo.btCollisionObjectWrapper);
            let objA = Ammo.castObject(colWrapper0.getCollisionObject(), Ammo.btRigidBody);

            let colWrapper1 = Ammo.wrapPointer(colObj1Wrap, Ammo.btCollisionObjectWrapper);
            let objB = Ammo.castObject(colWrapper1.getCollisionObject(), Ammo.btRigidBody);

            let index, localPos, worldPos

            if (objA.index != bodyA.index) {

                index = objA.index;
                //	localPos = contactPoint.get_m_localPointA();
                worldPos = contactPoint.get_m_positionWorldOnA();

            } else {

                index = objB.index;
                //	localPos = contactPoint.get_m_localPointB();
                worldPos = contactPoint.get_m_positionWorldOnB();

            }

            if (index == indexB) {
                let pos = {
                    x: worldPos.x(),
                    y: worldPos.y(),
                    z: worldPos.z()
                };
                contact.copy(pos);
            }
        }
    }

    let cbContactPairResult;

    function setupContactPair(contact) {

        cbContactPairResult = new AmmoLib.ConcreteContactResultCallback();

        cbContactPairResult.hasContact = false;

        cbContactPairResult.addSingleResult = function(cp, colObj0Wrap, partId0, index0, colObj1Wrap, partId1, index1) {

            let contactPoint = Ammo.wrapPointer(cp, Ammo.btManifoldPoint);

            const distance = contactPoint.getDistance();

            if (distance > 0) {

                let worldPos = contactPoint.get_m_positionWorldOnA();
                let pos = {
                    x: worldPos.x(),
                    y: worldPos.y(),
                    z: worldPos.z()
                };
                contact.copy(pos);
                contact.c = 1;
            } else {
                contact.c = 0;
            }
        }

    }

    //-------------------------------------------------------------------
    //contacts functions
    //-------------------------------------------------------------------

    //contact between two meshes
    function getContact(meshA, meshB, contact) {

        const bodyA = meshMap.get(meshA);
        let indexB = meshB.index;

        if (!cbContactResult) {
            setupContact(bodyA, indexB, contact);
        }

        world.contactTest(bodyA, cbContactResult);
    }

    //all contacts on target mesh
    function getContactPair(index, contact) {

        const body = meshMap.get(index);

        //init
        if (!cbContactPairResult) {
            setupContactPair(contact);
        }

        world.contactTest(body, cbContactPairResult);
    }

    function getAllCollisions(mesh, markers) {

        let meshIndex = mesh.index;

        let dispatcher = world.getDispatcher();
        let numManifolds = dispatcher.getNumManifolds();

        for (let i = 0; i < numManifolds; i++) {

            let contactManifold = dispatcher.getManifoldByIndexInternal(i);

            let numContacts = contactManifold.getNumContacts();

            for (let j = 0; j < numContacts; j++) {

                let contactPoint = contactManifold.getContactPoint(j);
                let distance = contactPoint.getDistance();

                if (distance > 0) {

                    let objA = Ammo.castObject(contactManifold.getBody0(), Ammo.btRigidBody);
                    let objB = Ammo.castObject(contactManifold.getBody1(), Ammo.btRigidBody);

                    //	body     : objA
                    //	id 		 : objA.id
                    //	velocity : objA.getLinearVelocity()
                    //	worldPos : contactPoint.get_m_positionWorldOnA()
                    //	localPos : contactPoint.get_m_localPointA()

                    //only display contact from other meshes

                    let index, worldPos;

                    if (objA.index != meshIndex) {

                        index = objA.index;
                        worldPos = contactPoint.get_m_positionWorldOnA();

                        if (objB.index != meshIndex) {
                            index = 'fail'; //no contact with target
                        }

                    } else if (objB.index != meshIndex) {

                        index = objB.index;
                        worldPos = contactPoint.get_m_positionWorldOnB();
                    }


                    if (markers[index]) {

                        let pos = {
                            x: worldPos.x(),
                            y: worldPos.y(),
                            z: worldPos.z()
                        };
                        markers[objA.index].position.copy(pos);
                        markers[objA.index].visible = true;
                    }

                }
            }
        }
    }

    //

    let lastTime = 0;

    function step() {

        const time = performance.now();

        if (lastTime > 0) {

            const delta = (time - lastTime) / 1000;

            // console.time( 'world.step' );
            world.stepSimulation(delta, 10);
            // console.timeEnd( 'world.step' );

        }

        lastTime = time;

        //		

        for (let i = 0, l = meshes.length; i < l; i++) {

            const mesh = meshes[i];

            if (mesh.isInstancedMesh) {

                const array = mesh.instanceMatrix.array;
                const bodies = meshMap.get(mesh);


                for (let j = 0; j < bodies.length; j++) {

                    if (bodies[j].isActive()) {

                        const body = bodies[j];

                        const motionState = body.getMotionState();
                        motionState.getWorldTransform(transform);

                        const position = transform.getOrigin();
                        const quaternion = transform.getRotation();

                        compose(position, quaternion, array, j * 16);

                        mesh.instanceMatrix.needsUpdate = true;

                    }
                }

            } else {

                const body = meshMap.get(mesh);

                if (body.isActive()) {

                    const motionState = body.getMotionState();
                    motionState.getWorldTransform(transform);

                    const position = transform.getOrigin();
                    const quaternion = transform.getRotation();
                    mesh.position.set(position.x(), position.y(), position.z());
                    mesh.quaternion.set(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

                }
            }
        }
    }

    // animate
    setInterval(step, 1000 / frameRate);

    return {
        addMesh: addMesh,
        setMeshPosition: setMeshPosition,
        setMeshVelocity: setMeshVelocity,
        getAllCollisions: getAllCollisions,
        getContact: getContact,
        getContactPair: getContactPair
    };
}

function compose(position, quaternion, array, index) {

    const x = quaternion.x(),
        y = quaternion.y(),
        z = quaternion.z(),
        w = quaternion.w();
    const x2 = x + x,
        y2 = y + y,
        z2 = z + z;
    const xx = x * x2,
        xy = x * y2,
        xz = x * z2;
    const yy = y * y2,
        yz = y * z2,
        zz = z * z2;
    const wx = w * x2,
        wy = w * y2,
        wz = w * z2;

    array[index + 0] = (1 - (yy + zz));
    array[index + 1] = (xy + wz);
    array[index + 2] = (xz - wy);
    array[index + 3] = 0;

    array[index + 4] = (xy - wz);
    array[index + 5] = (1 - (xx + zz));
    array[index + 6] = (yz + wx);
    array[index + 7] = 0;

    array[index + 8] = (xz + wy);
    array[index + 9] = (yz - wx);
    array[index + 10] = (1 - (xx + yy));
    array[index + 11] = 0;

    array[index + 12] = position.x();
    array[index + 13] = position.y();
    array[index + 14] = position.z();
    array[index + 15] = 1;

}

export {
    AmmoPhysics
};