import * as THREE from 'three';

import { GUI } from 'three/addons/libs/lil-gui.module.min.js';

import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { ShaderPass } from 'three/addons/postprocessing/ShaderPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { OutputPass } from 'three/addons/postprocessing/OutputPass.js';

const BLOOM_SCENE = 1;

const bloomLayer = new THREE.Layers();
bloomLayer.set( BLOOM_SCENE );

const params = {
  threshold: 0,
  strength: 1,
  //radius: 0.5,
  radius: 0,
  //exposure: 1
  exposure: 0.7,
  moonlight: false,
  scale: 10,
  applyColormap: false,
  droneColor: 0x00ff00
};

const darkMaterial = new THREE.MeshBasicMaterial( { color: 'black' } );
const materials = {};

const renderer = new THREE.WebGLRenderer( { antialias: true } );
renderer.setPixelRatio( window.devicePixelRatio );
renderer.setSize( window.innerWidth, window.innerHeight );
renderer.toneMapping = THREE.ReinhardToneMapping;
renderer.toneMappingExposure = Math.pow( params.exposure, 4.0 );
renderer.setAnimationLoop( animate );
renderer.shadowMap.enabled = true;
document.body.appendChild( renderer.domElement );

const scene = new THREE.Scene();

//const camera = new THREE.PerspectiveCamera( 40, window.innerWidth / window.innerHeight, 1, 200 );
const camera = new THREE.PerspectiveCamera( 40, window.innerWidth / window.innerHeight, 1, 1000 );
camera.position.set( -75, 45, 75 );
camera.lookAt( 0, 100, 0 );

const controls = new OrbitControls( camera, renderer.domElement );
//controls.maxPolarAngle = Math.PI * 0.5;
controls.maxPolarAngle = Math.PI * 0.5 - 0.05;
controls.minDistance = 1;
//controls.maxDistance = 100;
controls.maxDistance = 200;
controls.addEventListener( 'change', render );

const renderScene = new RenderPass( scene, camera );

const bloomPass = new UnrealBloomPass( new THREE.Vector2( window.innerWidth, window.innerHeight ), 1.5, 0.4, 0.85 );
bloomPass.threshold = params.threshold;
bloomPass.strength = params.strength;
bloomPass.radius = params.radius;

const bloomComposer = new EffectComposer( renderer );
bloomComposer.renderToScreen = false;
bloomComposer.addPass( renderScene );
bloomComposer.addPass( bloomPass );

const mixPass = new ShaderPass(
  new THREE.ShaderMaterial( {
    uniforms: {
      baseTexture: { value: null },
      bloomTexture: { value: bloomComposer.renderTarget2.texture }
    },
    vertexShader: document.getElementById( 'vertexshader' ).textContent,
    fragmentShader: document.getElementById( 'fragmentshader' ).textContent,
    defines: {}
  } ), 'baseTexture'
);
mixPass.needsSwap = true;

const outputPass = new OutputPass();

const finalComposer = new EffectComposer( renderer );
finalComposer.addPass( renderScene );
finalComposer.addPass( mixPass );
finalComposer.addPass( outputPass );

const raycaster = new THREE.Raycaster();

const mouse = new THREE.Vector2();

//window.addEventListener( 'pointerdown', onPointerDown );

// Swarm params

let num_drones = 12;
let drones = [];

let sphere = new THREE.Mesh();
let moonmesh = new THREE.Mesh();
let moonline = new THREE.Line();

let drone_scale = params.scale;
let drone_color = params.droneColor;

const gui = new GUI();

const bloomFolder = gui.addFolder( 'bloom' );

bloomFolder.add( params, 'threshold', 0.0, 1.0 ).onChange( function ( value ) {

  bloomPass.threshold = Number( value );
  render();

} );

bloomFolder.add( params, 'strength', 0.0, 3 ).onChange( function ( value ) {

  bloomPass.strength = Number( value );
  render();

} );

bloomFolder.add( params, 'radius', 0.0, 1.0 ).step( 0.01 ).onChange( function ( value ) {

  bloomPass.radius = Number( value );
  render();

} );

bloomFolder.add( bloomPass, 'enabled' );

const toneMappingFolder = gui.addFolder( 'tone mapping' );

toneMappingFolder.add( params, 'exposure', 0.1, 2 ).onChange( function ( value ) {

  renderer.toneMappingExposure = Math.pow( value, 4.0 );
  render();

} );

const vizFolder = gui.addFolder( 'moonlight visualization' );

vizFolder.add( params, 'moonlight' ).onChange( function ( value ) {

  moonmesh.material.opacity = value;
  moonline.material.opacity = value;
  render();

} );

vizFolder.add( params, 'scale', 0.1, 20 ).onChange( function ( value ) {

  drone_scale = value;

} );

vizFolder.add( params, 'applyColormap' ).onChange( function ( value ) {
  for (let i = 0; i < num_drones; i++) {
    if (value) {
      drones[i].material.color.setHSL( i/num_drones, 0.7, 0.5 );
    } else {
      drones[i].material.color.setHex( drone_color ); 
    }
  } 
} ); 

vizFolder.addColor( params, 'droneColor' ).onChange( function ( value ) {

  drone_color = value;
  
  for (let i = 0; i < num_drones; i++) {
    drones[i].material.color.setHex( drone_color ); 
  } 

} );


setupScene();

function animate() {
  
  // Move sphere
  //sphere.position.x = Math.sin( Date.now() * 0.001 ) * 15;
  //sphere.position.z = Math.cos( Date.now() * 0.001 ) * 15;
  
  // Move all drones
  /*for (let i = 0; i < num_drones; i++) {
    drones[i].position.x = Math.sin( Date.now() * 0.001 + (6.28/(num_drones))*i ) * 10;
    drones[i].position.z = Math.cos( Date.now() * 0.001 + (6.28/(num_drones))*i ) * 10;
    //drones[i].position.y = Math.cos( Date.now() * 0.001 + (6.28/(num_drones))*i ) * 3 + 10;

    //drones[i].position.x = Math.sin( 0.001 + (6.28/(num_drones))*i ) * 10;
    //drones[i].position.z = Math.cos( 0.001 + (6.28/(num_drones))*i ) * 10;
    //drones[i].position.x = Math.sin( 0.001 + (6.28/(num_drones-1))*i ) * 10;
    //drones[i].position.z = Math.cos( 0.001 + (6.28/(num_drones-1))*i ) * 10;
  }*/


  render();

}

function onPointerDown( event ) {

  mouse.x = ( event.clientX / window.innerWidth ) * 2 - 1;
  mouse.y = - ( event.clientY / window.innerHeight ) * 2 + 1;

  raycaster.setFromCamera( mouse, camera );
  const intersects = raycaster.intersectObjects( scene.children, false );
  if ( intersects.length > 0 ) {

    const object = intersects[ 0 ].object;
    object.layers.toggle( BLOOM_SCENE );
    render();

  }

}

window.onresize = function () {

  const width = window.innerWidth;
  const height = window.innerHeight;

  camera.aspect = width / height;
  camera.updateProjectionMatrix();

  renderer.setSize( width, height );

  bloomComposer.setSize( width, height );
  finalComposer.setSize( width, height );

  render();

};

function setupScene() {

  /*scene.traverse( disposeMaterial );
  scene.children.length = 0;

  const geometry = new THREE.IcosahedronGeometry( 1, 15 );

  for ( let i = 0; i < 50; i ++ ) {

    const color = new THREE.Color();
    color.setHSL( Math.random(), 0.7, Math.random() * 0.2 + 0.05 );

    const material = new THREE.MeshBasicMaterial( { color: color } );
    const sphere = new THREE.Mesh( geometry, material );
    sphere.position.x = Math.random() * 10 - 5;
    sphere.position.y = Math.random() * 10 - 5;
    sphere.position.z = Math.random() * 10 - 5;
    sphere.position.normalize().multiplyScalar( Math.random() * 4.0 + 2.0 );
    sphere.scale.setScalar( Math.random() * Math.random() + 0.5 );
    scene.add( sphere );

    if ( Math.random() < 0.25 ) sphere.layers.enable( BLOOM_SCENE );

  }

  */

  // Skybox
  const skyboxGeometry = new THREE.SphereGeometry( 500, 60, 40 );
  skyboxGeometry.scale( -1, 1, 1 );
  const texture = new THREE.TextureLoader().load( 'pano.jpg' );

  //texture.colorSpace = THREE.SRGBColorSpace;
  const skyboxMaterial = new THREE.MeshBasicMaterial( { map: texture } );
  const skybox = new THREE.Mesh( skyboxGeometry, skyboxMaterial );
  scene.add( skybox );
  skybox.rotation.y = Math.PI;

  // Add moonlight (directional)
  let moon = new THREE.Vector3( 40, 40, 0 );

  const directionalLight = new THREE.DirectionalLight( 0xffffff, 1 );
  directionalLight.position.set( moon.x, moon.y, moon.z );
  directionalLight.target.position.set( 0, 0, 0 );
  directionalLight.castShadow = true;
  //directionalLight.castShadow = false;
  scene.add( directionalLight );
  scene.add( directionalLight.target );
  directionalLight.shadow.camera.near = 0.1;
  directionalLight.shadow.camera.far = 100;
  directionalLight.shadow.camera.left = -50;
  directionalLight.shadow.camera.right = 50;
  directionalLight.shadow.camera.top = 50;
  directionalLight.shadow.camera.bottom = -50;

  // Add moon indicator
  const moonGeometry = new THREE.SphereGeometry( 1, 32, 32 );
  const moonMaterial = new THREE.MeshBasicMaterial( { color: 0xffffff } );
  moonMaterial.transparent = true;
  moonMaterial.opacity = 0.25;
  moonmesh = new THREE.Mesh( moonGeometry, moonMaterial );
  moonmesh.position.set( moon.x, moon.y, moon.z );
  scene.add( moonmesh );
  moonmesh.material.opacity = 0;
  moonmesh.layers.enable( BLOOM_SCENE );

  // Add moonlight indicator (line from 40, 20, 0 to 0, 0, 0)
  let moonmag = Math.sqrt( moon.x*moon.x + moon.y*moon.y + moon.z*moon.z );
  const lineGeometry = new THREE.CylinderGeometry( 0.1, 0.1, moonmag, 32 );
  // Make line go from 0, 0, 0 to moon.x, moon.y, moon.z
  lineGeometry.translate( 0, moonmag/2, 0 ); 
  let quat = new THREE.Quaternion();
  quat.setFromUnitVectors( new THREE.Vector3( 0, 1, 0 ), new THREE.Vector3( moon.x, moon.y, moon.z ).normalize() );
  lineGeometry.applyQuaternion( quat );
  
  const lineMaterial = new THREE.LineBasicMaterial( { color: 0xffffff } );
  lineMaterial.transparent = true;
  lineMaterial.opacity = 0.25;
  moonline = new THREE.Line( lineGeometry, lineMaterial );
  moonline.material.opacity = 0;
  scene.add( moonline );

  /*
  const geometry = new THREE.SphereGeometry( 1, 32, 32 );
  const material = new THREE.MeshBasicMaterial( { color: 0xffff00 } );
  material.transparent = true;
  //material.opacity = 0.15;
  material.opacity = 1;
  sphere = new THREE.Mesh( geometry, material );
  sphere.castShadow = true;  

  scene.add( sphere );
  sphere.position.set( 0, 2, 0 ); 
  
  sphere.layers.enable( BLOOM_SCENE );*/

  // Add drones
  for (let i = 0; i < num_drones; i++) {
    const droneGeometry = new THREE.SphereGeometry( 1, 32, 32 );
    const droneMaterial = new THREE.MeshBasicMaterial( { color: 0x00ff00 } );
    const droneMesh = new THREE.Mesh( droneGeometry, droneMaterial );
    droneMesh.castShadow = true;
    droneMesh.position.set( Math.random() * 10 - 5, 15, Math.random() * 10 - 5 );
    droneMesh.layers.enable( BLOOM_SCENE );
    drones.push( droneMesh );
    scene.add( droneMesh );
  }

  // Create floor
  //const floorGeometry = new THREE.PlaneGeometry( 100, 100, 1, 1 );
  //const floorGeometry = new THREE.PlaneGeometry( 500, 500, 1, 1 );
  //const floorMaterial = new THREE.MeshPhongMaterial( { color: 0x808080 } );
  const floorTexture = new THREE.TextureLoader().load( 'grass.jpg' );
  floorTexture.wrapS = THREE.RepeatWrapping;
  floorTexture.wrapT = THREE.RepeatWrapping;
  floorTexture.repeat.set( 10, 10 );
  const floorMaterial = new THREE.MeshStandardMaterial( { map: floorTexture } );

  const floorGeometry = new THREE.CylinderGeometry( 500, 500, 1, 32 );
  const floor = new THREE.Mesh( floorGeometry, floorMaterial );
  //floor.rotation.x = -Math.PI / 2;
  scene.add( floor );

  floor.receiveShadow = true;

  // Add telemetry setInterval
  setInterval( getTelemetry, 10 );

  render();

}

function disposeMaterial( obj ) {

  if ( obj.material ) {

    obj.material.dispose();

  }

}

function render() {

  scene.traverse( darkenNonBloomed );
  bloomComposer.render();
  scene.traverse( restoreMaterial );

  // render the entire scene, then render bloom scene on top
  finalComposer.render();

}

function getTelemetry() {
  let response = fetch('http://localhost:19001/stream')
    .then(response => response.json())
    .then(data => {
      for (let i = 0; i < num_drones; i++) {
	// Search for "tello_" + i
	let current_drone = "tello_" + i
	drones[i].position.x = data[current_drone].x*drone_scale;
	drones[i].position.z = data[current_drone].y*drone_scale;
	drones[i].position.y = data[current_drone].z*drone_scale;
      }
  });

}

function darkenNonBloomed( obj ) {

  if ( obj.isMesh && bloomLayer.test( obj.layers ) === false ) {

    materials[ obj.uuid ] = obj.material;
    obj.material = darkMaterial;

  }

}

function restoreMaterial( obj ) {

  if ( materials[ obj.uuid ] ) {

    obj.material = materials[ obj.uuid ];
    delete materials[ obj.uuid ];

  }

}
