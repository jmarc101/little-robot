/* Team Jean --
 Jean-Marc Prud'homme (20137035)
 Jean-Daniel Toupin
 TP1 IFT 3355 -  MEGATRON

* */


THREE.Object3D.prototype.setMatrix = function(a) {
  this.matrix = a;
  this.matrix.decompose(this.position, this.quaternion, this.scale);
};

var start = Date.now();
// SETUP RENDERER AND SCENE
var scene = new THREE.Scene();
var renderer = new THREE.WebGLRenderer();
renderer.setClearColor(0xffffff); // white background colour
document.body.appendChild(renderer.domElement);

// SETUP CAMERA
var camera = new THREE.PerspectiveCamera(30, 1, 0.1, 1000); // view angle, aspect ratio, near, far
camera.position.set(10,5,10);
camera.lookAt(scene.position);
scene.add(camera);

// SETUP ORBIT CONTROL OF THE CAMERA
var controls = new THREE.OrbitControls(camera);
controls.damping = 0.2;

// ADAPT TO WINDOW RESIZE
function resize() {
  renderer.setSize(window.innerWidth, window.innerHeight);
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
}

window.addEventListener('resize', resize);
resize();

// FLOOR WITH CHECKERBOARD
var floorTexture = new THREE.ImageUtils.loadTexture('images/tile.jpg');
floorTexture.wrapS = floorTexture.wrapT = THREE.MirroredRepeatWrapping;
floorTexture.repeat.set(4, 4);

var floorMaterial = new THREE.MeshBasicMaterial({ map: floorTexture, side: THREE.DoubleSide });
var floorGeometry = new THREE.PlaneBufferGeometry(15, 15);
var floor = new THREE.Mesh(floorGeometry, floorMaterial);
floor.rotation.x = Math.PI / 2;
floor.position.y = 0.0;
scene.add(floor);

// TRANSFORMATIONS

/**
 * multiply 2 matrix together
 * @param m1
 * @param m2
 * @returns multiplied matrices
 */
function multMat(m1, m2){
  return idMat4().multiplyMatrices(m1, m2);
}

/**
 * matrix inversion
 * @param m
 * @returns inverted matrix
 */
function inverseMat(m){
  return idMat4().getInverse(m, true);
}

/**
 * Identity matrix
 * @returns matrix of identity 4
 */
function idMat4(){

  return new THREE.Matrix4().set(1,0,0,0,
                                 0,1,0,0,
                                 0,0,1,0,
                                 0,0,0,1)


}

/**
 * Matrix translation
 * @param matrix   matrix to translate
 * @param x       x movement
 * @param y       y movement
 * @param z       z movement
 * @returns {*}   changed matrix
 */
function translateMat(matrix, x, y, z){
  // Apply translation [x, y, z] to @matrix
  // matrix: THREE.Matrix3
  // x, y, z: float
  var matrixTrans = idMat4().set(1,0,0,x,
                                 0,1,0,y,
                                 0,0,1,z,
                                 0,0,0,1)

  return multMat(matrixTrans, matrix);

}

/**
 * Matrix rotation fucntion
 * @param matrix  matrix to change
 * @param angle     angle of rotation
 * @param axis    which axis of rotation
 * @returns {*}   rotated Matrix
 */
function rotateMat(matrix, angle, axis){
  // Apply rotation by @angle with respect to @axis to @matrix
  // matrix: THREE.Matrix3
  // angle: float
  // axis: string "x", "y" or "z"
  var matrixRotate;
  if (axis === "x") {
    matrixRotate = idMat4().set(1,0,0,0,
                                0,Math.cos(angle),-(Math.sin(angle)),0,
                                0,Math.sin(angle),Math.cos(angle),0,
                                 0,0,0,1);
  } else if (axis === "y") {
    matrixRotate = idMat4().set(Math.cos(angle),0,Math.sin(angle),0,
                               0,1,0,0,
                                (-(Math.sin(angle))),0,Math.cos(angle),0,
                               0,0,0,1);
  } else {
    matrixRotate = idMat4().set(Math.cos(angle),(-(Math.sin(angle))),0,0,
                              (Math.sin(angle)),(Math.cos(angle)),0,0,
                              0,0,1,0,
                              0,0,0,1);
  }

  return multMat(matrixRotate, matrix);

}

/**
 * Rotate vectors
 * @param v   vector rotate
 * @param angle angle of rotation
 * @param axis  rotation-axis
 * @returns {*} returns the rotated vector
 */
function rotateVec3(v, angle, axis){
  // Apply rotation by @angle with respect to @axis to vector @v
  //   // v: THREE.Vector3
  //   // angle: float
  //   // axis: string "x", "y" or "z"
  var matrixRotate;

  if (axis === "x") {
    matrixRotate = new THREE.Matrix3().set(1,0,0,
                                        0,Math.cos(angle),(-(Math.sin(angle))),
                                          0,Math.sin(angle),Math.cos(angle))
  } else if (axis === "y") {
    matrixRotate = new THREE.Matrix3().set(Math.cos(angle),0,Math.sin(angle),
                                          0,1,0,
                                          (-(Math.sin(angle))),0,Math.cos(angle))
  } else {
    matrixRotate = new THREE.Matrix3().set(Math.cos(angle),(-(Math.sin(angle))),0,
                                          (Math.sin(angle)),(Math.cos(angle)),0,
                                          0,0,1)
  }

  return v.applyMatrix3(matrixRotate);
}

function rescaleMat(matrix, x, y, z){
  // Apply scaling @x, @y and @z to @matrix
  // matrix: THREE.Matrix3
  // x, y, z: float

  var matrixScale = idMat4().set(x,0,0,0,
                                  0,y,0,0,
                                  0,0,z,0,
                                  0,0,0,1);

  return multMat(matrixScale, matrix);

}

/**
 * Our main Robot class
 */
class Robot {

  armAngleZ = Math.PI/2;
  armAngleX = 0;
  foreArmAngleLeft = 0;
  foreArmAngleRight = 0;
  legAngle = 0;
  leftFeetAngle = 0;
  rightFeetAngle = 0;
  leftUp = true;
  walkSpeed = .03;
  floorVal =  Math.PI/2;
  floorVal2 = 0;


  constructor() {
    // Geometry
    this.torsoHeight = 1.5;
    this.torsoRadius = 0.75;
    this.headRadius = 0.32;
    this.armRightRadius = 0.3;
    this.armLeftRadius = 0.3;
    this.foreArmRightRadius = 0.3;
    this.foreArmLeftRadius = 0.3;
    this.legHeight = 0.25;
    this.feetHeight = 0.25;
    this.shoeHeight = .4

    this.hatRadius = 0.5







    // Animation
    this.walkDirection = new THREE.Vector3( 0, 0, 1 );

    // Material
    this.material = new THREE.MeshNormalMaterial();

    // Initial pose
    this.initialize()
  }

  initialTorsoMatrix(){
    var initialTorsoMatrix = idMat4();
    initialTorsoMatrix = translateMat(initialTorsoMatrix, 0,this.torsoHeight +  (4*this.legHeight) , 0);

    return initialTorsoMatrix;
  }

  initialHeadMatrix(){
    var initialHeadMatrix = idMat4();
    initialHeadMatrix = translateMat(initialHeadMatrix, 0, this.torsoHeight/2 + this.headRadius, 0);

    return initialHeadMatrix;
  }

  initialHatMatrix(){
    var initialHatMatrix = idMat4();
    initialHatMatrix =  translateMat(initialHatMatrix, 0, this.torsoHeight/2 + this.hatRadius + 0.2, 0);

    return initialHatMatrix;
  }

  initialTopHatMatrix(){
    var initialTopHatMatrix = idMat4();
    initialTopHatMatrix =  translateMat(initialTopHatMatrix, 0, this.torsoHeight/2 + this.hatRadius + 0.5, 0);

    return initialTopHatMatrix;
  }


  initialLeftLegMatrix(){

    var initialLeftLegMatrix = rescaleMat(idMat4(),2, 1.5,1.5);
    initialLeftLegMatrix = rotateMat(initialLeftLegMatrix,Math.PI/2,"z");
    initialLeftLegMatrix = translateMat(initialLeftLegMatrix,
        -this.torsoRadius/2,-this.torsoHeight/2 -  this.feetHeight*2, 0);

    return initialLeftLegMatrix;
  }

  initialRightLegMatrix(){

    var initialRightLegMatrix = rescaleMat(idMat4(),2, 1.5,1.5);
    initialRightLegMatrix = rotateMat(initialRightLegMatrix,Math.PI/2,"z");
    initialRightLegMatrix = translateMat(initialRightLegMatrix,
        this.torsoRadius/2,-this.torsoHeight/2 -  this.feetHeight*2, 0);

    return initialRightLegMatrix;
  }

  initialLeftFeetMatrix(){

    var initialLeftFeetMatrix = rescaleMat(idMat4(),2,1,1);
    initialLeftFeetMatrix = rotateMat(initialLeftFeetMatrix,Math.PI/2,"z");
    initialLeftFeetMatrix = translateMat(initialLeftFeetMatrix,
        -this.torsoRadius/2,-this.torsoHeight- this.feetHeight *2, 0);

    return initialLeftFeetMatrix;
  }

  initialRightFeetMatrix(){

    var initialRightFeetMatrix = rescaleMat(idMat4(),2,1,1);
    initialRightFeetMatrix = rotateMat(initialRightFeetMatrix,Math.PI/2,"z");
    initialRightFeetMatrix = translateMat(initialRightFeetMatrix,
        this.torsoRadius/2,-this.torsoHeight- this.feetHeight *2, 0);

    return initialRightFeetMatrix;
  }


  initialArmRightMatrix(){
    var initialArmRightMatrix = idMat4();
    initialArmRightMatrix = rescaleMat(initialArmRightMatrix, 1.5,1,.5)
    initialArmRightMatrix = rotateMat(initialArmRightMatrix,(-Math.PI/2), "z")
    initialArmRightMatrix = translateMat(initialArmRightMatrix,
        this.torsoRadius + this.armRightRadius,
        this.torsoRadius/2, 0);

    return initialArmRightMatrix;
  }

  initialArmLeftMatrix(){
    var initialArmLeftMatrix = idMat4();
    initialArmLeftMatrix = rescaleMat(initialArmLeftMatrix, 1.5,1,.5)
    initialArmLeftMatrix = rotateMat(initialArmLeftMatrix,Math.PI/2, "z")
    initialArmLeftMatrix = translateMat(initialArmLeftMatrix,
        -(this.torsoRadius + this.armLeftRadius),
        this.torsoRadius/2, 0);

    return initialArmLeftMatrix;
  }

  initialRightForeArmMatrix(){
    var initialRightForeArmMatrix = idMat4();
    initialRightForeArmMatrix = rescaleMat(initialRightForeArmMatrix, 1.5,.75,.5)
    initialRightForeArmMatrix = rotateMat(initialRightForeArmMatrix,(-Math.PI/2), "z")
    initialRightForeArmMatrix = translateMat(initialRightForeArmMatrix,
        (this.torsoRadius + this.foreArmLeftRadius),
        (-this.torsoRadius/2), 0);

    return initialRightForeArmMatrix;
  }

  initialLeftForeArmMatrix(){
    var initialLeftForeArmMatrix = idMat4();
    initialLeftForeArmMatrix = rescaleMat(initialLeftForeArmMatrix, 1.5,.75,.5)
    initialLeftForeArmMatrix = rotateMat(initialLeftForeArmMatrix,Math.PI/2, "z")
    initialLeftForeArmMatrix = translateMat(initialLeftForeArmMatrix,
        -(this.torsoRadius + this.foreArmLeftRadius),
        (-this.torsoRadius/2), 0);

    return initialLeftForeArmMatrix;
  }

  initialLeftShoeMatrix(){
    return translateMat(idMat4(), -this.torsoRadius/2,
        -(this.torsoHeight +  (4*this.legHeight)) + this.legHeight/2,this.legHeight/2 );
  }

  initialrightShoeMatrix(){
    return translateMat(idMat4(), this.torsoRadius/2,
        -(this.torsoHeight +  (4*this.legHeight)) + this.legHeight/2,this.legHeight/2 );
  }


  initialize() {
    // Torso
    var torsoGeometry = new THREE.CubeGeometry(2*this.torsoRadius, this.torsoHeight, this.torsoRadius, 64);
    this.torso = new THREE.Mesh(torsoGeometry, this.material);

    // Head
    var headGeometry = new THREE.SphereGeometry(this.headRadius,8,6,0,Math.PI * 2,0,Math.PI)
    this.head = new THREE.Mesh(headGeometry, this.material);

    //Hat
    var hatGeometry  = new THREE.CylinderGeometry(.45,this.hatRadius, 0.1,50);
    var topHatGeometry = new THREE.CylinderGeometry(0,1, .5,64,64);

    this.topHatGeometry = new THREE.Mesh(topHatGeometry, this.material);
    this.topHatGeometry.setMatrix(this.initialTopHatMatrix())
    hatGeometry.merge(topHatGeometry);

    this.hat = new THREE.Mesh(hatGeometry, this.material);


    // arms
    var armGeometry = new THREE.SphereGeometry(this.armRightRadius,8,6,0,Math.PI * 2,0,Math.PI);
    this.armRight = new THREE.Mesh(armGeometry, this.material);
    this.armLeft = new THREE.Mesh(armGeometry, this.material);

    // ForeArms
    var foreArmGerometry = new THREE.SphereGeometry(this.foreArmRightRadius,8,6,0,Math.PI * 2,0,Math.PI);
    this.rightForeArm = new THREE.Mesh(foreArmGerometry, this.material);
    this.leftForeArm = new THREE.Mesh(foreArmGerometry, this.material);

    //  Legs
    var legGeometry = new THREE.SphereGeometry(this.legHeight,8,6,0,Math.PI * 2,0,Math.PI);
    this.leftLeg = new THREE.Mesh(legGeometry, this.material);
    this.rightLeg = new THREE.Mesh(legGeometry, this.material);

    //  Feet
    var feetGeometry = new THREE.SphereGeometry(this.feetHeight,8,6,0,Math.PI * 2,0,Math.PI);
    this.leftFeet = new THREE.Mesh(feetGeometry, this.material);
    this.rightFeet = new THREE.Mesh(feetGeometry, this.material);

    //shoes
    var shoeGeometry = new THREE.BoxGeometry(this.shoeHeight,this.shoeHeight,2*this.shoeHeight,)
    this.leftShoe = new THREE.Mesh(shoeGeometry, this.material)
    this.rightShoe = new THREE.Mesh(shoeGeometry, this.material)


    // *** TRANSFORMATION ***

    // Torso transformation
    this.torsoInitialMatrix = this.initialTorsoMatrix();
    this.torsoMatrix = idMat4();
    this.torso.setMatrix(this.torsoInitialMatrix);

    // Head transformation
    this.headInitialMatrix = this.initialHeadMatrix();
    this.headMatrix = idMat4();
    var matrixHead = multMat(this.torsoInitialMatrix, this.headInitialMatrix);
    this.head.setMatrix(matrixHead);

    // Hat transformation
    this.hatInitialMatrix = this.initialHatMatrix();
    this.hatMatrix = idMat4();
    var matrixHat = multMat(this.torsoInitialMatrix, this.hatInitialMatrix);
    this.hat.setMatrix(matrixHat)

    // Arm left Transformation
    this.armLeftInitialMatrix = this.initialArmLeftMatrix();
    this.armLeftMatrix = idMat4();
    var matrixLeftArm = multMat(this.torsoInitialMatrix, this.armLeftInitialMatrix);
    this.armLeft.setMatrix(matrixLeftArm);

    // right foreArm Transformation
    this.rightForeArmInitMatrix = this.initialRightForeArmMatrix();
    this.rightForeArmMatrix = idMat4();
    var rightForeArmMatrix = multMat(this.torsoInitialMatrix, this.rightForeArmInitMatrix);
    this.rightForeArm.setMatrix(rightForeArmMatrix);

    // Left foreArm Transformation
    this.leftForeArmInitMatrix = this.initialLeftForeArmMatrix();
    this.leftForeArmMatrix = idMat4();
    var leftForeArmMatrix = multMat(this.torsoInitialMatrix, this.leftForeArmInitMatrix);
    this.leftForeArm.setMatrix(leftForeArmMatrix);

    // Arm right Transformation
    this.armRightInitialMatrix = this.initialArmRightMatrix();
    this.armRightMatrix = idMat4();
    var matrixRightArm = multMat(this.torsoInitialMatrix, this.armRightInitialMatrix);
    this.armRight.setMatrix(matrixRightArm);

    // Left Leg transormation
    this.leftLegInitialMatrix = this.initialLeftLegMatrix();
    this.leftLegMatrix = idMat4();
    var matrixLeftLeg = multMat(this.torsoInitialMatrix, this.leftLegInitialMatrix);
    this.leftLeg.setMatrix(matrixLeftLeg);

    // Right Leg transformation
    this.rightLegInitialMatrix = this.initialRightLegMatrix();
    this.rightLegMatrix = idMat4();
    var matrixRightLeg = multMat(this.torsoInitialMatrix, this.rightLegInitialMatrix);
    this.rightLeg.setMatrix(matrixRightLeg);

    // Left feet transormation
    this.leftFeetInitialMatrix = this.initialLeftFeetMatrix();
    this.leftFeetMatrix = idMat4();
    var matrixLeftFeet = multMat(this.torsoInitialMatrix, this.leftFeetInitialMatrix);
    this.leftFeet.setMatrix(matrixLeftFeet);

    // Right feet transormation
    this.rightFeetInitialMatrix = this.initialRightFeetMatrix();
    this.rightFeetMatrix = idMat4();
    var matrixRightFeet = multMat(this.torsoInitialMatrix, this.rightFeetInitialMatrix);
    this.rightFeet.setMatrix(matrixRightFeet);

    //shoes
    this.leftShoeInitMatrix = this.initialLeftShoeMatrix();
    this.leftShoeMatrix = idMat4();
    var matrixLeftShoe = multMat(this.torsoInitialMatrix, this.leftShoeInitMatrix);
    this.leftShoe.setMatrix(matrixLeftShoe)

    this.rightShoeInitMatrix = this.initialrightShoeMatrix();
    this.rightShoeMatrix = idMat4();
    var matrixRightShoe = multMat(this.torsoInitialMatrix, this.rightShoeInitMatrix);
    this.rightShoe.setMatrix(matrixRightShoe)

	// Add robot to scene
	scene.add(this.torso);
    scene.add(this.head);
    scene.add(this.hat)
    scene.add(this.armRight);
    scene.add(this.leftLeg);
    scene.add(this.leftFeet)
    scene.add(this.armLeft);
    scene.add(this.rightForeArm);
    scene.add(this.leftForeArm);
    scene.add(this.rightLeg);
    scene.add(this.rightFeet);

  }



  /**
   * Rotation of Torso, everytime you add a bodypart, need to add it to fonction
   * @param axis axis of rotation
   * @param angle radian - rotation on y axis
   */
  rotateTorso(axis, angle){
    var torsoMatrix = this.torsoMatrix;

    this.torsoMatrix = idMat4();
    this.torsoMatrix = rotateMat(this.torsoMatrix, angle, axis);
    this.torsoMatrix = multMat(torsoMatrix, this.torsoMatrix);

    var matrixTorso = multMat(this.torsoMatrix, this.torsoInitialMatrix);
    this.torso.setMatrix(matrixTorso);

    var matrixHead = multMat(this.headMatrix, this.headInitialMatrix);
    var matrixRotateHead = multMat(matrixTorso, matrixHead);
    this.head.setMatrix(matrixRotateHead);

    var matrixHat = multMat(this.hatMatrix, this.hatInitialMatrix);
    var matrixRotateHat = multMat(matrixTorso,matrixHat);
    this.hat.setMatrix(matrixRotateHat);

    var matrixArmRight = multMat(this.armRightMatrix, this.armRightInitialMatrix);
    var matrixRotArm = multMat(matrixTorso, matrixArmRight);
    this.armRight.setMatrix(matrixRotArm);

    var matrixLeftArm = multMat(this.armLeftMatrix, this.armLeftInitialMatrix);
    var matrixRotLeftArm = multMat(matrixTorso, matrixLeftArm);
    this.armLeft.setMatrix(matrixRotLeftArm);

    var matrixLeftForeArm = multMat(this.leftForeArmMatrix, this.leftForeArmInitMatrix);
    matrixLeftForeArm = multMat(this.armLeftMatrix, matrixLeftForeArm);
    matrixLeftForeArm = multMat(matrixTorso, matrixLeftForeArm);
    this.leftForeArm.setMatrix(matrixLeftForeArm);

    var matrixForeArmright = multMat(this.rightForeArmMatrix, this.rightForeArmInitMatrix);
    matrixForeArmright = multMat(this.armRightMatrix, matrixForeArmright)
    matrixForeArmright = multMat(matrixTorso, matrixForeArmright);
    this.rightForeArm.setMatrix(matrixForeArmright);


    this.walkDirection = rotateVec3(this.walkDirection, angle, "y");


    var leftLegMatrix = multMat(this.leftLegMatrix, this.leftLegInitialMatrix);
    leftLegMatrix = multMat(matrixTorso,leftLegMatrix);
    this.leftLeg.setMatrix(leftLegMatrix);

    var rightLegMatrix = multMat(this.rightLegMatrix, this.rightLegInitialMatrix);
    rightLegMatrix = multMat(matrixTorso,rightLegMatrix);
    this.rightLeg.setMatrix(rightLegMatrix);

    var leftFeetMatrix = multMat(this.leftFeetMatrix, this.leftFeetInitialMatrix);
    leftFeetMatrix = multMat(this.leftLegMatrix, leftFeetMatrix);
    leftFeetMatrix = multMat(matrixTorso,leftFeetMatrix);
    this.leftFeet.setMatrix(leftFeetMatrix);

    var rightFeetMatrix = multMat(this.rightFeetMatrix, this.rightFeetInitialMatrix);
    rightFeetMatrix = multMat(this.rightLegMatrix, rightFeetMatrix);
    rightFeetMatrix = multMat(matrixTorso,rightFeetMatrix);
    this.rightFeet.setMatrix(rightFeetMatrix);


  }

  /**
   * Moves torso forward and backwards on xz plane
   * @param speed
   */
  moveTorso(axis, speed){

    if (axis === "y"){
      this.torsoMatrix = translateMat(this.torsoMatrix, 0,speed,0);
    }

    this.torsoMatrix = translateMat(this.torsoMatrix, speed * this.walkDirection.x, speed * this.walkDirection.y, speed * this.walkDirection.z);

    var matrixTorso = multMat(this.torsoMatrix, this.torsoInitialMatrix);
    this.torso.setMatrix(matrixTorso);

    var matrixHead = multMat(this.headMatrix, this.headInitialMatrix);
    var matrixTranslateHead = multMat(matrixTorso, matrixHead);
    this.head.setMatrix(matrixTranslateHead);

    var matrixHat = multMat(this.hatMatrix, this.hatInitialMatrix);
    var matrixTranslateHat = multMat(matrixTorso, matrixHat);
    this.hat.setMatrix(matrixTranslateHat);

    var leftLegMatrix = multMat(this.leftLegMatrix, this.leftLegInitialMatrix);
    leftLegMatrix = multMat(matrixTorso, leftLegMatrix);
    this.leftLeg.setMatrix(leftLegMatrix);

    var rightLegMatrix = multMat(this.rightLegMatrix, this.rightLegInitialMatrix);
    rightLegMatrix = multMat(matrixTorso, rightLegMatrix);
    this.rightLeg.setMatrix(rightLegMatrix);

    var leftFeetMatrix = multMat(this.leftFeetMatrix, this.leftFeetInitialMatrix);
    leftFeetMatrix = multMat(this.leftLegMatrix, leftFeetMatrix);
    leftFeetMatrix = multMat(matrixTorso, leftFeetMatrix);
    this.leftFeet.setMatrix(leftFeetMatrix);

    var rightFeetMatrix = multMat(this.rightFeetMatrix, this.rightFeetInitialMatrix);
    rightFeetMatrix = multMat(this.rightLegMatrix, rightFeetMatrix);
    rightFeetMatrix = multMat(matrixTorso, rightFeetMatrix);
    this.rightFeet.setMatrix(rightFeetMatrix);

    var matrixArmRight = multMat(this.armRightMatrix, this.armRightInitialMatrix);
    var matrixTransArm = multMat(matrixTorso, matrixArmRight);
    this.armRight.setMatrix(matrixTransArm);

    var matrixArmLeft = multMat(this.armLeftMatrix, this.armLeftInitialMatrix);
    var matrixTransArmLeft = multMat(matrixTorso, matrixArmLeft);
    this.armLeft.setMatrix(matrixTransArmLeft);

    var matrixForeArmLeft = multMat(this.leftForeArmMatrix, this.leftForeArmInitMatrix);
    matrixForeArmLeft = multMat(this.armLeftMatrix, matrixForeArmLeft)
    matrixForeArmLeft = multMat(matrixTorso, matrixForeArmLeft);
    this.leftForeArm.setMatrix(matrixForeArmLeft);

    var matrixForeArmright = multMat(this.rightForeArmMatrix, this.rightForeArmInitMatrix);
    matrixForeArmright = multMat(this.armRightMatrix, matrixForeArmright)
    matrixForeArmright = multMat(matrixTorso, matrixForeArmright);
    this.rightForeArm.setMatrix(matrixForeArmright);

  }


  /**
   * Rotate head independent of rest of torso
   * @param angle  y-axis rotation
   */
  rotateHead(angle){
    var headMatrix = this.headMatrix;

    this.headMatrix = idMat4();
    this.headMatrix = rotateMat(this.headMatrix, angle, "y");
    this.headMatrix = multMat(headMatrix, this.headMatrix);

    var matrix = multMat(this.headMatrix, this.headInitialMatrix);
    matrix = multMat(this.torsoMatrix, matrix);
    matrix = multMat(this.torsoInitialMatrix, matrix);
    this.head.setMatrix(matrix);

    var matrixHat = multMat(this.hatMatrix, this.hatInitialMatrix);
    matrixHat = multMat(this.headMatrix, matrixHat);
    matrixHat = multMat(this.torsoMatrix, matrixHat);
    matrixHat = multMat(this.torsoInitialMatrix, matrixHat);
    this.hat.setMatrix(matrixHat);
  }

  /**
   * Rotate hat independent of rest of torso
   * @param angle y-axis rotation
   */
  rotateHat(angle){
    var hatMatrix = this.hatMatrix;

    this.hatMatrix = idMat4();
    this.hatMatrix = rotateMat(this.hatMatrix, angle, "y");
    this.hatMatrix = multMat(hatMatrix, this.hatMatrix);

    var matrixHat = multMat(this.hatMatrix, this.hatInitialMatrix);
    matrixHat = multMat(this.torsoMatrix, matrixHat);
    matrixHat = multMat(this.torsoInitialMatrix, matrixHat);
    this.hat.setMatrix(matrixHat);

  }


  /**
   * Rotate fore arms independent of rest of arms
   * they rotate symmetrically
   * @param foreArm which forearm to transform
   * @param angle   x-axis rotation
   */
  rotateForeArm(foreArm, angle){



    if (foreArm === "right"){

      if (this.foreArmAngleRight <=0 && angle <= 0){
        return;
      }

      var rightForeArmTemp = this.rightForeArmMatrix;


      this.rightForeArmMatrix = rotateMat(idMat4(), -angle, "x");
      this.rightForeArmMatrix = multMat(rightForeArmTemp, this.rightForeArmMatrix);

      var matrixRight = multMat(this.rightForeArmMatrix, this.rightForeArmInitMatrix);
      matrixRight = multMat(this.armRightMatrix, matrixRight)
      matrixRight = multMat(this.torsoMatrix, matrixRight);
      matrixRight = multMat(this.torsoInitialMatrix, matrixRight);
      this.rightForeArm.setMatrix(matrixRight);

      this.foreArmAngleRight += angle;

    } else {

      if (this.foreArmAngleLeft <=0 && angle <= 0){
        return;
      }

      var leftForeArmTemp = this.leftForeArmMatrix;

      this.leftForeArmMatrix = rotateMat(idMat4(),-angle, "x")
      this.leftForeArmMatrix = multMat(leftForeArmTemp, this.leftForeArmMatrix);


      var matrixLeft = multMat(this.leftForeArmMatrix, this.leftForeArmInitMatrix);
      matrixLeft = multMat(this.armLeftMatrix, matrixLeft)
      matrixLeft = multMat(this.torsoMatrix,  matrixLeft);
      matrixLeft = multMat(this.torsoInitialMatrix, matrixLeft);
      this.leftForeArm.setMatrix(matrixLeft);

      this.foreArmAngleLeft += angle;

    }

  }

  /**
   * Rotate arms independent of torso, forearms will follow arms
   * @param arm which arm to rotate
   * @param axis axis of rotation
   * @param angle x,z Axis-rotation
   */
  rotateArm(arm, axis, angle){
    var rightArmTemp = this.armRightMatrix;
    var leftArmTemp = this.armLeftMatrix;

    if (arm === "left"){


      // Move to axis 0,0 before rotation
      this.armLeftMatrix = translateMat(idMat4(),(this.torsoRadius),(-this.torsoRadius), 0 )
      this.armLeftMatrix = rotateMat(this.armLeftMatrix, angle, axis);
      this.armLeftMatrix = translateMat(this.armLeftMatrix,-(this.torsoRadius),this.torsoRadius, 0 );
      this.armLeftMatrix = multMat(leftArmTemp, this.armLeftMatrix);


      // leftArm
      var matrixleft = multMat(this.armLeftMatrix, this.armLeftInitialMatrix);
      matrixleft = multMat(this.torsoMatrix, matrixleft);
      matrixleft = multMat(this.torsoInitialMatrix, matrixleft);
      this.armLeft.setMatrix(matrixleft);

      //foreArm left
      var matLeftForeArm = multMat(this.leftForeArmMatrix, this.leftForeArmInitMatrix);
      matLeftForeArm = multMat(this.armLeftMatrix, matLeftForeArm);
      matLeftForeArm = multMat(this.torsoMatrix, matLeftForeArm);
      matLeftForeArm = multMat(this.torsoInitialMatrix, matLeftForeArm);
      this.leftForeArm.setMatrix(matLeftForeArm);
    }

    else{

      // Move to axis 0,0 before rotation
      this.armRightMatrix = translateMat(idMat4(),-(this.torsoRadius),(-this.torsoRadius), 0 )
      this.armRightMatrix = rotateMat(this.armRightMatrix, angle, axis);
      this.armRightMatrix = translateMat(this.armRightMatrix,(this.torsoRadius),this.torsoRadius, 0 );
      this.armRightMatrix = multMat(rightArmTemp, this.armRightMatrix);

      // rightArm
      var matrixRight = multMat(this.armRightMatrix, this.armRightInitialMatrix);
      matrixRight = multMat(this.torsoMatrix, matrixRight);
      matrixRight = multMat(this.torsoInitialMatrix, matrixRight);
      this.armRight.setMatrix(matrixRight);


      // foreArm right
      var matrixRightForeArm = multMat(this.rightForeArmMatrix, this.rightForeArmInitMatrix);
      matrixRightForeArm = multMat(this.armRightMatrix, matrixRightForeArm);
      matrixRightForeArm = multMat(this.torsoMatrix, matrixRightForeArm);
      matrixRightForeArm = multMat(this.torsoInitialMatrix, matrixRightForeArm);
      this.rightForeArm.setMatrix(matrixRightForeArm);

    }
  }

  rotateLegs(leg, angle){

    var leftLegTemp = this.leftLegMatrix;
    var rightLegTemp = this.rightLegMatrix;


    if (leg === "left"){
      this.leftLegMatrix = translateMat(idMat4(), 0,this.torsoHeight/2 ,0 );
      this.leftLegMatrix = rotateMat(this.leftLegMatrix, -angle, "x");
      this.leftLegMatrix = translateMat(this.leftLegMatrix,0, -this.torsoHeight/2, 0 );
      this.leftLegMatrix = multMat(leftLegTemp, this.leftLegMatrix );

      var matrixleftLeg = multMat(this.leftLegMatrix, this.leftLegInitialMatrix);
      matrixleftLeg = multMat(this.torsoMatrix, matrixleftLeg);
      matrixleftLeg = multMat(this.torsoInitialMatrix, matrixleftLeg);
      this.leftLeg.setMatrix(matrixleftLeg);

      //leftFeet left
      var matLeftFeet = multMat(this.leftFeetMatrix, this.leftFeetInitialMatrix);
      matLeftFeet = multMat(this.leftLegMatrix, matLeftFeet);
      matLeftFeet = multMat(this.torsoMatrix, matLeftFeet);
      matLeftFeet = multMat(this.torsoInitialMatrix, matLeftFeet);
      this.leftFeet.setMatrix(matLeftFeet);


    } else {

      this.rightLegMatrix = translateMat(idMat4(), 0,this.torsoHeight/2 ,0 );
      this.rightLegMatrix = rotateMat(this.rightLegMatrix, angle, "x");
      this.rightLegMatrix = translateMat(this.rightLegMatrix,0, -this.torsoHeight/2, 0 )
      this.rightLegMatrix = multMat(rightLegTemp, this.rightLegMatrix );

      var matrixrightLeg = multMat(this.rightLegMatrix, this.rightLegInitialMatrix);
      matrixrightLeg = multMat(this.torsoMatrix, matrixrightLeg);
      matrixrightLeg = multMat(this.torsoInitialMatrix, matrixrightLeg);
      this.rightLeg.setMatrix(matrixrightLeg);

      var matRightFeet = multMat(this.rightFeetMatrix, this.rightFeetInitialMatrix);
      matRightFeet = multMat(this.rightLegMatrix, matRightFeet);
      matRightFeet = multMat(this.torsoMatrix, matRightFeet);
      matRightFeet = multMat(this.torsoInitialMatrix, matRightFeet);
      this.rightFeet.setMatrix(matRightFeet);
    }
  }

  rotateFeet(feet,  angle){
    var leftTemp = this.leftFeetMatrix;
    var rightTemp = this.rightFeetMatrix;

    if (feet === "left"){


      this.leftFeetMatrix = translateMat(idMat4(), 0,this.torsoHeight   ,0 );
      this.leftFeetMatrix = rotateMat(this.leftFeetMatrix, angle, "x");
      this.leftFeetMatrix = translateMat(this.leftFeetMatrix, 0,-this.torsoHeight ,0 );
      this.leftFeetMatrix = multMat(leftTemp, this.leftFeetMatrix);

      var feetLegMat = multMat(this.leftFeetMatrix, this.leftFeetInitialMatrix);
      feetLegMat = multMat(this.leftLegMatrix, feetLegMat)
      feetLegMat = multMat(this.torsoMatrix, feetLegMat);
      feetLegMat = multMat(this.torsoInitialMatrix, feetLegMat);
      this.leftFeet.setMatrix(feetLegMat);

      robot.leftFeetAngle += angle;

    } else {



      this.rightFeetMatrix = translateMat(idMat4(), 0,this.torsoHeight  ,0 );
      this.rightFeetMatrix = rotateMat(this.rightFeetMatrix, angle, "x");
      this.rightFeetMatrix = translateMat(this.rightFeetMatrix, 0,-this.torsoHeight ,0 );
      this.rightFeetMatrix = multMat(rightTemp, this.rightFeetMatrix);

      var rightFeetMat = multMat(this.rightFeetMatrix, this.rightFeetInitialMatrix);
      rightFeetMat = multMat(this.rightLegMatrix, rightFeetMat)
      rightFeetMat = multMat(this.torsoMatrix, rightFeetMat);
      rightFeetMat = multMat(this.torsoInitialMatrix, rightFeetMat);
      this.rightFeet.setMatrix(rightFeetMat);

      robot.rightFeetAngle += angle;
    }
  }



}

var robot = new Robot();

// LISTEN TO KEYBOARD
var keyboard = new THREEx.KeyboardState();

var selectedRobotComponent = 0;
var components = [
  "Torso",
  "Head",
  "Hat",
  "Arms",
  "Fore Arms",
  "legs",
  "feet"
];
var numberComponents = components.length;

function checkKeyboard() {
  // Next element
  if (keyboard.pressed("e")){
    selectedRobotComponent = selectedRobotComponent + 1;

    if (selectedRobotComponent<0){
      selectedRobotComponent = numberComponents - 1;
    }

    if (selectedRobotComponent >= numberComponents){
      selectedRobotComponent = 0;
    }

    window.alert(components[selectedRobotComponent] + " selected");
  }

  // Previous element
  if (keyboard.pressed("q")){
    selectedRobotComponent = selectedRobotComponent - 1;

    if (selectedRobotComponent < 0){
      selectedRobotComponent = numberComponents - 1;
    }

    if (selectedRobotComponent >= numberComponents){
      selectedRobotComponent = 0;
    }

    window.alert(components[selectedRobotComponent] + " selected");
  }

  // UP
  if (keyboard.pressed("w")){
    switch (components[selectedRobotComponent]) {
      case "Torso":


        //Move Torso
        robot.moveTorso(null,this.robot.walkSpeed);



        let walk = robot.walkSpeed * Math.sign(Math.sin(robot.floorVal));
        let float = -.009 *  Math.sin(robot.floorVal2)/.75;

        robot.moveTorso("y", float);



        robot.rotateLegs("left",  walk);
        robot.rotateLegs("right",  walk);

        // y-axis for hip movement
        robot.rotateTorso("y",walk/7);

        //Arms
        robot.rotateArm("left", "x", walk/2);
        robot.rotateArm("right", "x", -walk/2);

        // foreArm
        robot.rotateForeArm("left", -walk/2)
        robot.rotateForeArm("right", walk/2)

        // Feet

        robot.rotateFeet("left",  -float );
        robot.rotateFeet("right", -float );


        robot.floorVal += Math.PI / 40;
        robot.floorVal2 += Math.PI/ 20;





        break;

      case "Head":
        break;

      case "Hat":
        break;

      case "Fore Arms" :
        if (robot.foreArmAngleLeft >= 2.2){
          break;
        }
        robot.rotateForeArm("left", .1);
        robot.rotateForeArm("right", .1);
        robot.foreArmAngleLeft +=.1;
        robot.foreArmAngleRight +=.1;

        break;
      case "Arms" :

        if (robot.armAngleX >= 1.8) {
          break;
        }
        robot.rotateArm("left","x",-0.1);
        robot.rotateArm("right","x",-0.1);
        robot.armAngleX += .1;
        break;
      case "legs":

        if (robot.legAngle >= .8 ){
          break;
        }

        robot.rotateLegs("left", .1);
        robot.rotateLegs("right", .1);
        robot.legAngle += .1;
        break;

      case "feet":

        robot.rotateFeet("left",0.1)
        robot.rotateFeet("right",0.1)
        break;

    }
  }

  // DOWN
  if (keyboard.pressed("s")){
    switch (components[selectedRobotComponent]){


      case "Torso":
        robot.moveTorso(null, -Math.abs(robot.walkSpeed/2));



        let walk = -robot.walkSpeed *  Math.sign(Math.sin(robot.floorVal));
        let float = .009 *  Math.sin(robot.floorVal2)/.75;

        robot.moveTorso("y", float);



        robot.rotateLegs("left",  walk);

        robot.rotateLegs("right",  walk);

        // y-axis for hip movement
        robot.rotateTorso("y",walk/7);

        //Arms
        robot.rotateArm("left", "x", walk/2);
        robot.rotateArm("right", "x", -walk/2);

        // foreArm
        robot.rotateForeArm("left", -walk/2)
        robot.rotateForeArm("right", walk/2)



        robot.floorVal -= Math.PI / 40;
        robot.floorVal2 -= Math.PI/ 20;

        break;



      case "Head":
        break;

      case "Hat":
        break;


      case "Fore Arms" :
        if (robot.foreArmAngleLeft <= 0){
          break;
        }
        robot.rotateForeArm("left",-0.1);
        robot.rotateForeArm("right",-0.1);
        robot.foreArmAngleLeft -=.1;
        robot.foreArmAngleRight -=.1;
        break;


      case "Arms" :
      if (robot.armAngleX <= -.3) {
        break;
      }
      robot.rotateArm("left","x",0.1);
      robot.rotateArm("right","x",0.1);
      robot.armAngleX -= .1;
      break;

      case "legs":
        if (robot.legAngle <= -.8 ){
          break;
        }
        robot.rotateLegs("left", -.1);
        robot.rotateLegs("right", -.1);
        robot.legAngle -=.1;
        break;

      case "feet":
        robot.rotateFeet("left",-0.1)
        robot.rotateFeet("right",-0.1)
        break;
    }
  }

  // LEFT
  if (keyboard.pressed("a")) {
    switch (components[selectedRobotComponent]) {
      case "Torso":
        robot.rotateTorso("y", 0.1);
        break;
      case "Head":
        robot.rotateHead(0.1);

        break;
      case "Hat":
        robot.rotateHat(0.1)
        break;
      case "Arms" :

        if (robot.armAngleZ <= .55){
          break;
        }
          robot.rotateArm("left", "z", -0.1);
          robot.rotateArm("right", "z", 0.1);
          robot.armAngleZ -= 0.1;
          break;

    }
  }

  // RIGHT
  if (keyboard.pressed("d")){
    switch (components[selectedRobotComponent]) {
      case "Torso":
        robot.rotateTorso("y", -0.1);
        break;
      case "Head":
        robot.rotateHead(-0.1);

        break;
      case "Hat":
        robot.rotateHat(-0.1);
        break;
      case "Arms" :

      if (robot.armAngleZ >= 1.58){
        break;
      }

          robot.rotateArm("left", "z", 0.1);
          robot.rotateArm("right", "z", -0.1);
          robot.armAngleZ += 0.1;
          break;

    }
  }
}

// SETUP UPDATE CALL-BACK
function update() {
  checkKeyboard();
  requestAnimationFrame(update);
  renderer.render(scene, camera);
}

update();