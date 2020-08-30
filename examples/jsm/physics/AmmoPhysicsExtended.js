async function AmmoPhysics() {

	if ( 'Ammo' in window === false ) {

		console.error( 'AmmoPhysics: Couldn\'t find Ammo.js' );
		return;

	}

	const AmmoLib = await Ammo();

	const frameRate = 60;
	const collisionConfiguration = new AmmoLib.btDefaultCollisionConfiguration();
	const dispatcher = new AmmoLib.btCollisionDispatcher( collisionConfiguration );
	const broadphase = new AmmoLib.btDbvtBroadphase();
	const solver = new AmmoLib.btSequentialImpulseConstraintSolver();
	const world = new AmmoLib.btDiscreteDynamicsWorld( dispatcher, broadphase, solver, collisionConfiguration );
	world.setGravity( new AmmoLib.btVector3( 0, - 9.8, 0 ) );

	const worldTransform = new AmmoLib.btTransform();

	//

	function getShape( geometry ) {

		const parameters = geometry.parameters;

		// TODO change type to is*

		if ( geometry.type === 'BoxBufferGeometry' ) {

			const sx = parameters.width !== undefined ? parameters.width / 2 : 0.5;
			const sy = parameters.height !== undefined ? parameters.height / 2 : 0.5;
			const sz = parameters.depth !== undefined ? parameters.depth / 2 : 0.5;

			const shape = new AmmoLib.btBoxShape( new AmmoLib.btVector3( sx, sy, sz ) );
			shape.setMargin( 0.05 );

			return shape;

		} else if ( geometry.type === 'PlaneBufferGeometry' ) {

			const sx = parameters.width !== undefined ? parameters.width / 2 : 0.5;
			const sy = parameters.height !== undefined ? parameters.height / 2 : 0.5;

			const shape = new AmmoLib.btBoxShape( new AmmoLib.btVector3( sx, sy, 0.01 ) );
			shape.setMargin( 0.05 );

			return shape;

		} else if ( geometry.type === 'SphereBufferGeometry' ) {

			const sr = parameters.radius !== undefined ? parameters.radius : 1;

			const shape = new AmmoLib.btSphereShape( sr );
			shape.setMargin( 0 );

			return shape;
		
		} else if ( geometry.type === 'CylinderBufferGeometry' && parameters.radiusTop === parameters.radiusBottom ) {

			const st = parameters.radiusTop !== undefined ? parameters.radiusTop : 0.5;
			const sy = parameters.height !== undefined ? parameters.height : 1;
			
			const shape = new AmmoLib.btCylinderShape(new AmmoLib.btVector3(st, sy* 0.5, st));
			shape.setMargin( 0.05 );

			return shape;
		
		} else if ( geometry.type === 'ConeBufferGeometry' ) {

			const sr = parameters.radius !== undefined ? parameters.radius : 1;
			const sy = parameters.height !== undefined ? parameters.height : 1;

			const shape = new AmmoLib.btConeShape( sr, sy );
			shape.setMargin( 0 );

			return shape;
		}

		//convex hull converter
			
		var verticesPos = geometry.getAttribute('position').array;
			
		let triangles = [];
		for ( let i = 0; i < verticesPos.length; i += 3 ) {
			triangles.push({ x:verticesPos[i], y:verticesPos[i+1], z:verticesPos[i+2] })
		}
			
		let triangle, triangle_mesh = new Ammo.btTriangleMesh;
			
		const shape = new AmmoLib.btConvexHullShape();
			
		var vectA = new AmmoLib.btVector3(0,0,0);
		var vectB = new AmmoLib.btVector3(0,0,0);
		var vectC = new AmmoLib.btVector3(0,0,0);
	
		for ( let i = 0; i < triangles.length-3; i += 3 ) {
				
			vectA.setX(triangles[i].x);
			vectA.setY(triangles[i].y);
			vectA.setZ(triangles[i].z);
			shape.addPoint(vectA,true);

			vectB.setX(triangles[i+1].x);
			vectB.setY(triangles[i+1].y);
			vectB.setZ(triangles[i+1].z);
			shape.addPoint(vectB,true);

			vectC.setX(triangles[i+2].x);
			vectC.setY(triangles[i+2].y);
			vectC.setZ(triangles[i+2].z);
			shape.addPoint(vectC,true);
				
			triangle_mesh.addTriangle( vectA, vectB, vectC, true );
		}
			
		shape.setMargin( 0 );
			
		return shape;

	}

	const meshes = [];
	const meshMap = new WeakMap();

	function addMesh( mesh, body, mass, index) {
		
		let shape;
		if ( body.isGroup ) {
			shape = getCompoundShape( body );
		} else {
			shape = getShape( body.geometry );
		}
		
		if ( shape !== null ) {

			if ( mesh.isInstancedMesh ) {

				handleInstancedMesh( mesh, mass, shape, index );

			} else {

				handleMesh( mesh, mass, shape, index );

			}
		}
	}
	
	function getCompoundShape( body ) {

		const shapes = [];
		const positions = [];
		const quaternions = [];
		
		body.traverse( function ( child ) {
			if (child.isMesh) {
				
				shapes.push(getShape( child.geometry ));
				positions.push (child.position);
				quaternions.push (child.quaternion);
			}
		});
		
		const compShape = new AmmoLib.btCompoundShape();
		
		for ( let i = 0; i < shapes.length; i++ ) {
		
			shapes[i].setMargin( 0 );
			let transform = meshTransform (positions[i], quaternions[i]);
			compShape.addChildShape( transform, shapes[i] );
		}	
		
		return compShape;
	}

	function handleMesh( mesh, mass, shape, index ) {

		const position = mesh.position;
		const quaternion = mesh.quaternion;

		const transform = meshTransform (position, quaternion);

		const motionState = new AmmoLib.btDefaultMotionState( transform );

		const localInertia = new AmmoLib.btVector3( 0, 0, 0 );
		shape.calculateLocalInertia( mass, localInertia );

		const rbInfo = new AmmoLib.btRigidBodyConstructionInfo( mass, motionState, shape, localInertia );

		const body = new AmmoLib.btRigidBody( rbInfo );
		
		body.index = index;
		body.setSleepingThresholds(0.0, 0.1);
		//body.setFriction( 4 );

		world.addRigidBody( body );

		if ( mass > 0 ) {

			meshes.push( mesh );
			meshMap.set( mesh, body );

		}
	}
	
	function meshTransform( position, quaternion ) {
		
		const transform = new AmmoLib.btTransform();
		transform.setIdentity();
		transform.setOrigin( new AmmoLib.btVector3( position.x, position.y, position.z ) );
		transform.setRotation( new AmmoLib.btQuaternion( quaternion.x, quaternion.y, quaternion.z, quaternion.w ) );
		
		return transform;
	}

	function handleInstancedMesh( mesh, mass, shape, index ) {

		const array = mesh.instanceMatrix.array;

		const bodies = [];

		for ( let i = 0; i < mesh.count; i ++ ) {

			const indexes = i * 16;

			const transform = new AmmoLib.btTransform();
			transform.setFromOpenGLMatrix( array.slice( indexes, indexes + 16 ) );

			const motionState = new AmmoLib.btDefaultMotionState( transform );

			const localInertia = new AmmoLib.btVector3( 0, 0, 0 );
			shape.calculateLocalInertia( mass, localInertia );

			const rbInfo = new AmmoLib.btRigidBodyConstructionInfo( mass, motionState, shape, localInertia );

			const body = new AmmoLib.btRigidBody( rbInfo );
			
			body.index = index;

			world.addRigidBody( body );

			bodies.push( body );

		}

		if ( mass > 0 ) {

			mesh.instanceMatrix.setUsage( 35048 ); // THREE.DynamicDrawUsage = 35048
			meshes.push( mesh );

			meshMap.set( mesh, bodies );

		}
	}

	function setMeshVelocity( mesh, velocity, angular, index ) {

		if ( mesh.isInstancedMesh ) {

			const bodies = meshMap.get( mesh );
			
			const body = bodies[ index ];
			
			body.setAngularVelocity( new AmmoLib.btVector3( angular.x, angular.y, angular.z ) );

			body.setLinearVelocity( new AmmoLib.btVector3( velocity.x, velocity.y, velocity.z ) );

		} else {

			const body = meshMap.get( mesh );
			
			body.setAngularVelocity( new AmmoLib.btVector3( angular.x, angular.y, angular.z) );

			body.setLinearVelocity( new AmmoLib.btVector3( velocity.x, velocity.y, velocity.z ) );

		}

	}
	
	function setMeshPosition( mesh, position, quaternion, index ) {

		if ( mesh.isInstancedMesh ) {

			const bodies = meshMap.get( mesh );
			const body = bodies[ index ];
		
			worldTransform.setIdentity();
			worldTransform.setOrigin( new AmmoLib.btVector3( position.x, position.y, position.z ) );

			worldTransform.setRotation( new AmmoLib.btQuaternion( quaternion.x, quaternion.y, quaternion.z, quaternion.w ) );
			body.setWorldTransform( worldTransform );

		} else {

			const body = meshMap.get( mesh );

			worldTransform.setIdentity();
			worldTransform.setOrigin( new AmmoLib.btVector3( position.x, position.y, position.z ) );

			worldTransform.setRotation( new AmmoLib.btQuaternion( quaternion.x, quaternion.y, quaternion.z, quaternion.w ) );
			body.setWorldTransform( worldTransform );

		}
	}
	
	//
		
	let cbContactResult;
	
	function setupContact( bodyA, indexB, markers){

		cbContactResult = new AmmoLib.ConcreteContactResultCallback();
		
		cbContactResult.addSingleResult = function(cp, colObj0Wrap, partId0, index0, colObj1Wrap, partId1, index1){
			
			let contactPoint = Ammo.wrapPointer( cp, Ammo.btManifoldPoint );

			const distance = contactPoint.getDistance();

			if( distance > 0 ) return;

			let colWrapper0 = Ammo.wrapPointer( colObj0Wrap, Ammo.btCollisionObjectWrapper );
			let objA = Ammo.castObject( colWrapper0.getCollisionObject(), Ammo.btRigidBody );
			
			let colWrapper1 = Ammo.wrapPointer( colObj1Wrap, Ammo.btCollisionObjectWrapper );
			let objB = Ammo.castObject( colWrapper1.getCollisionObject(), Ammo.btRigidBody );

			let index, localPos, worldPos

			if( objA.index != bodyA.index ){

				index = objA.index;
				localPos = contactPoint.get_m_localPointA();
				worldPos = contactPoint.get_m_positionWorldOnA();

			}
			else{

				index = objB.index;
				localPos = contactPoint.get_m_localPointB();
				worldPos = contactPoint.get_m_positionWorldOnB();

			}

			if (index == indexB ){
			
			let pos = {x: worldPos.x(), y: 0.1, z: worldPos.z()};
			markers[0].position.copy(pos);
			markers[0].visible = true;

			}
		}
	}
	
	let cbContactPairResult;
	
	function setupContactPair(markers){

		cbContactPairResult = new AmmoLib.ConcreteContactResultCallback();
		
		cbContactPairResult.hasContact = false;

		cbContactPairResult.addSingleResult = function(cp, colObj0Wrap, partId0, index0, colObj1Wrap, partId1, index1){
			
			let contactPoint = Ammo.wrapPointer( cp, Ammo.btManifoldPoint );

			const distance = contactPoint.getDistance();

			if( distance > 0 ) {

			let worldPos = contactPoint.get_m_positionWorldOnA();
			let pos = {x: worldPos.x(), y: 0, z: worldPos.z()};
			markers[0].position.copy(pos);
			markers[0].visible = true;

			}
		}

	}

	function getContact( meshA, indexB, markers){
		
		const bodyA = meshMap.get( meshA );

		if ( ! cbContactResult ){
			setupContact(bodyA, indexB, markers);
		}

		world.contactTest( bodyA, cbContactResult );
	}
	
	function getContactPair(mesh, markers){

		const body = meshMap.get( mesh );
		
		if ( ! cbContactPairResult ){
			setupContactPair(markers);
		}

		world.contactTest( body, cbContactPairResult );
	}
	
	function getAllCollisions(meshes, markers) {	
	
		let dispatcher = world.getDispatcher();
        let numManifolds = dispatcher.getNumManifolds();

		for ( let i = 0; i < numManifolds; i ++ ) {

			let contactManifold = dispatcher.getManifoldByIndexInternal( i );
			
			let numContacts = contactManifold.getNumContacts();

			for ( let j = 0; j < numContacts; j++ ) {

				let contactPoint = contactManifold.getContactPoint( j );
				let distance = contactPoint.getDistance();

				if( distance > 0 ){
				
					let objA = Ammo.castObject( contactManifold.getBody0(), Ammo.btRigidBody );
					let objB = Ammo.castObject( contactManifold.getBody1(), Ammo.btRigidBody );
				
					//	body     : objA
					//	id 		 : objA.id
					//	velocity : objA.getLinearVelocity()
					//	worldPos : contactPoint.get_m_positionWorldOnA()
					//	localPos : contactPoint.get_m_localPointA()
					
					if (objA.index > -1 && objB.index < -1 && meshes[objA.index].contact == false){
						
						let worldPos = contactPoint.get_m_positionWorldOnA();
						let pos = {x: worldPos.x(), y: 0, z: worldPos.z()};
						markers[objA.index].position.copy(pos);
						markers[objA.index].visible = true;
						meshes[objA.index].contact = true;
					}
					
					if (objB.index > -1 && objA.index < -1 && meshes[objB.index].contact == false){
						
						let worldPos = contactPoint.get_m_positionWorldOnB();
						let pos = {x: worldPos.x(), y: 0, z: worldPos.z()};
						markers[objB.index].position.copy(pos);
						markers[objB.index].visible = true;
						meshes[objB.index].contact = true;
					}
					
				}
			}
		}
	}

	//

	let lastTime = 0;

	function step() {

		const time = performance.now();

		if ( lastTime > 0 ) {

			const delta = ( time - lastTime ) / 1000;

			// console.time( 'world.step' );
			world.stepSimulation( delta, 10 );
			// console.timeEnd( 'world.step' );

		}

		lastTime = time;

		//		

		for ( let i = 0, l = meshes.length; i < l; i ++ ) {

			const mesh = meshes[ i ];

			if ( mesh.isInstancedMesh ) {

				const array = mesh.instanceMatrix.array;
				const bodies = meshMap.get( mesh );

				for ( let j = 0; j < bodies.length; j ++ ) {

					const body = bodies[ j ];

					const motionState = body.getMotionState();
					motionState.getWorldTransform( worldTransform );

					const position = worldTransform.getOrigin();
					const quaternion = worldTransform.getRotation();

					compose( position, quaternion, array, j * 16 );

				}

				mesh.instanceMatrix.needsUpdate = true;

			} else {

				const body = meshMap.get( mesh );

				const motionState = body.getMotionState();
				motionState.getWorldTransform( worldTransform );

				const position = worldTransform.getOrigin();
				const quaternion = worldTransform.getRotation();
				mesh.position.set( position.x(), position.y(), position.z() );
				mesh.quaternion.set( quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w() );

			}

		}

	}

	// animate
	setInterval( step, 1000 / frameRate );

	return {
		addMesh: addMesh,
		setMeshPosition: setMeshPosition,
		setMeshVelocity: setMeshVelocity,
		getAllCollisions: getAllCollisions,
		getContact: getContact,
		getContactPair: getContactPair
	};
}

function compose( position, quaternion, array, index ) {

	const x = quaternion.x(), y = quaternion.y(), z = quaternion.z(), w = quaternion.w();
	const x2 = x + x, y2 = y + y, z2 = z + z;
	const xx = x * x2, xy = x * y2, xz = x * z2;
	const yy = y * y2, yz = y * z2, zz = z * z2;
	const wx = w * x2, wy = w * y2, wz = w * z2;

	array[ index + 0 ] = ( 1 - ( yy + zz ) );
	array[ index + 1 ] = ( xy + wz );
	array[ index + 2 ] = ( xz - wy );
	array[ index + 3 ] = 0;

	array[ index + 4 ] = ( xy - wz );
	array[ index + 5 ] = ( 1 - ( xx + zz ) );
	array[ index + 6 ] = ( yz + wx );
	array[ index + 7 ] = 0;

	array[ index + 8 ] = ( xz + wy );
	array[ index + 9 ] = ( yz - wx );
	array[ index + 10 ] = ( 1 - ( xx + yy ) );
	array[ index + 11 ] = 0;

	array[ index + 12 ] = position.x();
	array[ index + 13 ] = position.y();
	array[ index + 14 ] = position.z();
	array[ index + 15 ] = 1;

}

export { AmmoPhysics };