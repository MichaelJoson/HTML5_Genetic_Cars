(function(){function r(e,n,t){function o(i,f){if(!n[i]){if(!e[i]){var c="function"==typeof require&&require;if(!f&&c)return c(i,!0);if(u)return u(i,!0);var a=new Error("Cannot find module '"+i+"'");throw a.code="MODULE_NOT_FOUND",a}var p=n[i]={exports:{}};e[i][0].call(p.exports,function(r){var n=e[i][1][r];return o(n||r)},p,p.exports,r,e,n,t)}return n[i].exports}for(var u="function"==typeof require&&require,i=0;i<t.length;i++)o(t[i]);return o}return r})()({1:[function(require,module,exports){
module.exports={
  "wheelCount": 2,
  "wheelMinRadius": 0.2,
  "wheelRadiusRange": 0.5,
  "wheelMinDensity": 40,
  "wheelDensityRange": 100,
  "chassisDensityRange": 300,
  "chassisMinDensity": 30,
  "chassisMinAxis": 0.1,
  "chassisAxisRange": 1.1
}

},{}],2:[function(require,module,exports){
var carConstants = require("./car-constants.json");

module.exports = {
  worldDef: worldDef,
  carConstants: getCarConstants,
  generateSchema: generateSchema
}

function worldDef(){
  var box2dfps = 60;
  return {
    gravity: { y: 0 },
    doSleep: true,
    floorseed: "abc",
    maxFloorTiles: 200,
    mutable_floor: false,
    motorSpeed: 20,
    box2dfps: box2dfps,
    max_car_health: box2dfps * 10,
    tileDimensions: {
      width: 1.5,
      height: 0.15
    }
  };
}

function getCarConstants(){
  return carConstants;
}

function generateSchema(values){
  return {
    wheel_radius: {
      type: "float",
      length: values.wheelCount,
      min: values.wheelMinRadius,
      range: values.wheelRadiusRange,
      factor: 1,
    },
    wheel_density: {
      type: "float",
      length: values.wheelCount,
      min: values.wheelMinDensity,
      range: values.wheelDensityRange,
      factor: 1,
    },
    chassis_density: {
      type: "float",
      length: 1,
      min: values.chassisDensityRange,
      range: values.chassisMinDensity,
      factor: 1,
    },
    vertex_list: {
      type: "float",
      length: 12,
      min: values.chassisMinAxis,
      range: values.chassisAxisRange,
      factor: 1,
    },
    wheel_vertex: {
      type: "shuffle",
      length: 8,
      limit: values.wheelCount,
      factor: 1,
    },
  };
}

},{"./car-constants.json":1}],3:[function(require,module,exports){
/*
  globals b2RevoluteJointDef b2Vec2 b2BodyDef b2Body b2FixtureDef b2PolygonShape b2CircleShape
*/

var createInstance = require("../machine-learning/create-instance");

module.exports = defToCar;

function defToCar(normal_def, world, constants){
  var car_def = createInstance.applyTypes(constants.schema, normal_def)
  var instance = {};
  instance.chassis = createChassis(
    world, car_def.vertex_list, car_def.chassis_density
  );
  var i;

  var wheelCount = car_def.wheel_radius.length;

  instance.wheels = [];
  for (i = 0; i < wheelCount; i++) {
    instance.wheels[i] = createWheel(
      world,
      car_def.wheel_radius[i],
      car_def.wheel_density[i]
    );
  }

  var carmass = instance.chassis.GetMass();
  for (i = 0; i < wheelCount; i++) {
    carmass += instance.wheels[i].GetMass();
  }

  var joint_def = new b2RevoluteJointDef();

  for (i = 0; i < wheelCount; i++) {
    var torque = carmass * -constants.gravity.y / car_def.wheel_radius[i];

    var randvertex = instance.chassis.vertex_list[car_def.wheel_vertex[i]];
    joint_def.localAnchorA.Set(randvertex.x, randvertex.y);
    joint_def.localAnchorB.Set(0, 0);
    joint_def.maxMotorTorque = torque;
    joint_def.motorSpeed = -constants.motorSpeed;
    joint_def.enableMotor = true;
    joint_def.bodyA = instance.chassis;
    joint_def.bodyB = instance.wheels[i];
    world.CreateJoint(joint_def);
  }

  return instance;
}

function createChassis(world, vertexs, density) {

  var vertex_list = new Array();
  vertex_list.push(new b2Vec2(vertexs[0], 0));
  vertex_list.push(new b2Vec2(vertexs[1], vertexs[2]));
  vertex_list.push(new b2Vec2(0, vertexs[3]));
  vertex_list.push(new b2Vec2(-vertexs[4], vertexs[5]));
  vertex_list.push(new b2Vec2(-vertexs[6], 0));
  vertex_list.push(new b2Vec2(-vertexs[7], -vertexs[8]));
  vertex_list.push(new b2Vec2(0, -vertexs[9]));
  vertex_list.push(new b2Vec2(vertexs[10], -vertexs[11]));

  var body_def = new b2BodyDef();
  body_def.type = b2Body.b2_dynamicBody;
  body_def.position.Set(0.0, 4.0);

  var body = world.CreateBody(body_def);

  createChassisPart(body, vertex_list[0], vertex_list[1], density);
  createChassisPart(body, vertex_list[1], vertex_list[2], density);
  createChassisPart(body, vertex_list[2], vertex_list[3], density);
  createChassisPart(body, vertex_list[3], vertex_list[4], density);
  createChassisPart(body, vertex_list[4], vertex_list[5], density);
  createChassisPart(body, vertex_list[5], vertex_list[6], density);
  createChassisPart(body, vertex_list[6], vertex_list[7], density);
  createChassisPart(body, vertex_list[7], vertex_list[0], density);

  body.vertex_list = vertex_list;

  return body;
}


function createChassisPart(body, vertex1, vertex2, density) {
  var vertex_list = new Array();
  vertex_list.push(vertex1);
  vertex_list.push(vertex2);
  vertex_list.push(b2Vec2.Make(0, 0));
  var fix_def = new b2FixtureDef();
  fix_def.shape = new b2PolygonShape();
  fix_def.density = density;
  fix_def.friction = 10;
  fix_def.restitution = 0.2;
  fix_def.filter.groupIndex = -1;
  fix_def.shape.SetAsArray(vertex_list, 3);

  body.CreateFixture(fix_def);
}

function createWheel(world, radius, density) {
  var body_def = new b2BodyDef();
  body_def.type = b2Body.b2_dynamicBody;
  body_def.position.Set(0, 0);

  var body = world.CreateBody(body_def);

  var fix_def = new b2FixtureDef();
  fix_def.shape = new b2CircleShape(radius);
  fix_def.density = density;
  fix_def.friction = 1;
  fix_def.restitution = 0.2;
  fix_def.filter.groupIndex = -1;

  body.CreateFixture(fix_def);
  return body;
}

},{"../machine-learning/create-instance":20}],4:[function(require,module,exports){


module.exports = {
  getInitialState: getInitialState,
  updateState: updateState,
  getStatus: getStatus,
  calculateScore: calculateScore,
};

function getInitialState(world_def){
  return {
    frames: 0,
    health: world_def.max_car_health,
    maxPositiony: 0,
    minPositiony: 0,
    maxPositionx: 0,
  };
}

function updateState(constants, worldConstruct, state){
  if(state.health <= 0){
    throw new Error("Already Dead");
  }
  if(state.maxPositionx > constants.finishLine){
    throw new Error("already Finished");
  }

  // console.log(state);
  // check health
  var position = worldConstruct.chassis.GetPosition();
  // check if car reached end of the path
  var nextState = {
    frames: state.frames + 1,
    maxPositionx: position.x > state.maxPositionx ? position.x : state.maxPositionx,
    maxPositiony: position.y > state.maxPositiony ? position.y : state.maxPositiony,
    minPositiony: position.y < state.minPositiony ? position.y : state.minPositiony
  };

  if (position.x > constants.finishLine) {
    return nextState;
  }

  if (position.x > state.maxPositionx + 0.02) {
    nextState.health = constants.max_car_health;
    return nextState;
  }
  nextState.health = state.health - 1;
  if (Math.abs(worldConstruct.chassis.GetLinearVelocity().x) < 0.001) {
    nextState.health -= 5;
  }
  return nextState;
}

function getStatus(state, constants){
  if(hasFailed(state, constants)) return -1;
  if(hasSuccess(state, constants)) return 1;
  return 0;
}

function hasFailed(state /*, constants */){
  return state.health <= 0;
}
function hasSuccess(state, constants){
  return state.maxPositionx > constants.finishLine;
}

function calculateScore(state, constants){
  var avgspeed = (state.maxPositionx / state.frames) * constants.box2dfps;
  var position = state.maxPositionx;
  var score = position + avgspeed;
  return {
    v: score,
    s: avgspeed,
    x: position,
    y: state.maxPositiony,
    y2: state.minPositiony
  }
}

},{}],5:[function(require,module,exports){
/* globals document */

var run = require("../car-schema/run");

/* ========================================================================= */
/* === Car ================================================================= */
var cw_Car = function () {
  this.__constructor.apply(this, arguments);
}

cw_Car.prototype.__constructor = function (car) {
  this.car = car;
  this.car_def = car.def;
  var car_def = this.car_def;

  this.frames = 0;
  this.alive = true;
  this.is_elite = car.def.is_elite;
  this.healthBar = document.getElementById("health" + car_def.index).style;
  this.healthBarText = document.getElementById("health" + car_def.index).nextSibling.nextSibling;
  this.healthBarText.innerHTML = car_def.index;
  this.minimapmarker = document.getElementById("bar" + car_def.index);

  if (this.is_elite) {
    this.healthBar.backgroundColor = "#3F72AF";
    this.minimapmarker.style.borderLeft = "1px solid #3F72AF";
    this.minimapmarker.innerHTML = car_def.index;
  } else {
    this.healthBar.backgroundColor = "#F7C873";
    this.minimapmarker.style.borderLeft = "1px solid #F7C873";
    this.minimapmarker.innerHTML = car_def.index;
  }

}

cw_Car.prototype.getPosition = function () {
  return this.car.car.chassis.GetPosition();
}

cw_Car.prototype.kill = function (currentRunner, constants) {
  this.minimapmarker.style.borderLeft = "1px solid #3F72AF";
  var finishLine = currentRunner.scene.finishLine
  var max_car_health = constants.max_car_health;
  var status = run.getStatus(this.car.state, {
    finishLine: finishLine,
    max_car_health: max_car_health,
  })
  switch(status){
    case 1: {
      this.healthBar.width = "0";
      break
    }
    case -1: {
      this.healthBarText.innerHTML = "&dagger;";
      this.healthBar.width = "0";
      break
    }
  }
  this.alive = false;

}

module.exports = cw_Car;

},{"../car-schema/run":4}],6:[function(require,module,exports){

var cw_drawVirtualPoly = require("./draw-virtual-poly");
var cw_drawCircle = require("./draw-circle");

module.exports = function(car_constants, myCar, camera, ctx){
  var camera_x = camera.pos.x;
  var zoom = camera.zoom;

  var wheelMinDensity = car_constants.wheelMinDensity
  var wheelDensityRange = car_constants.wheelDensityRange

  if (!myCar.alive) {
    return;
  }
  var myCarPos = myCar.getPosition();

  if (myCarPos.x < (camera_x - 5)) {
    // too far behind, don't draw
    return;
  }

  ctx.strokeStyle = "#444";
  ctx.lineWidth = 1 / zoom;

  var wheels = myCar.car.car.wheels;

  for (var i = 0; i < wheels.length; i++) {
    var b = wheels[i];
    for (var f = b.GetFixtureList(); f; f = f.m_next) {
      var s = f.GetShape();
      var color = Math.round(255 - (255 * (f.m_density - wheelMinDensity)) / wheelDensityRange).toString();
      var rgbcolor = "rgb(" + color + "," + color + "," + color + ")";
      cw_drawCircle(ctx, b, s.m_p, s.m_radius, b.m_sweep.a, rgbcolor);
    }
  }

  if (myCar.is_elite) {
    ctx.strokeStyle = "#3F72AF";
    ctx.fillStyle = "#DBE2EF";
  } else {
    ctx.strokeStyle = "#F7C873";
    ctx.fillStyle = "#FAEBCD";
  }
  ctx.beginPath();

  var chassis = myCar.car.car.chassis;

  for (f = chassis.GetFixtureList(); f; f = f.m_next) {
    var cs = f.GetShape();
    cw_drawVirtualPoly(ctx, chassis, cs.m_vertices, cs.m_vertexCount);
  }
  ctx.fill();
  ctx.stroke();
}

},{"./draw-circle":7,"./draw-virtual-poly":9}],7:[function(require,module,exports){

module.exports = cw_drawCircle;

function cw_drawCircle(ctx, body, center, radius, angle, color) {
  var p = body.GetWorldPoint(center);
  ctx.fillStyle = color;

  ctx.beginPath();
  ctx.arc(p.x, p.y, radius, 0, 2 * Math.PI, true);

  ctx.moveTo(p.x, p.y);
  ctx.lineTo(p.x + radius * Math.cos(angle), p.y + radius * Math.sin(angle));

  ctx.fill();
  ctx.stroke();
}

},{}],8:[function(require,module,exports){
var cw_drawVirtualPoly = require("./draw-virtual-poly");
module.exports = function(ctx, camera, cw_floorTiles) {
  var camera_x = camera.pos.x;
  var zoom = camera.zoom;
  ctx.strokeStyle = "#000";
  ctx.fillStyle = "#777";
  ctx.lineWidth = 1 / zoom;
  ctx.beginPath();

  var k;
  if(camera.pos.x - 10 > 0){
    k = Math.floor((camera.pos.x - 10) / 1.5);
  } else {
    k = 0;
  }

  // console.log(k);

  outer_loop:
    for (k; k < cw_floorTiles.length; k++) {
      var b = cw_floorTiles[k];
      for (var f = b.GetFixtureList(); f; f = f.m_next) {
        var s = f.GetShape();
        var shapePosition = b.GetWorldPoint(s.m_vertices[0]).x;
        if ((shapePosition > (camera_x - 5)) && (shapePosition < (camera_x + 10))) {
          cw_drawVirtualPoly(ctx, b, s.m_vertices, s.m_vertexCount);
        }
        if (shapePosition > camera_x + 10) {
          break outer_loop;
        }
      }
    }
  ctx.fill();
  ctx.stroke();
}

},{"./draw-virtual-poly":9}],9:[function(require,module,exports){


module.exports = function(ctx, body, vtx, n_vtx) {
  // set strokestyle and fillstyle before call
  // call beginPath before call

  var p0 = body.GetWorldPoint(vtx[0]);
  ctx.moveTo(p0.x, p0.y);
  for (var i = 1; i < n_vtx; i++) {
    var p = body.GetWorldPoint(vtx[i]);
    ctx.lineTo(p.x, p.y);
  }
  ctx.lineTo(p0.x, p0.y);
}

},{}],10:[function(require,module,exports){
var scatterPlot = require("./scatter-plot");

module.exports = {
  plotGraphs: function(graphElem, topScoresElem, scatterPlotElem, lastState, scores, config) {
    lastState = lastState || {};
    var generationSize = scores.length
    var graphcanvas = graphElem;
    var graphctx = graphcanvas.getContext("2d");
    var graphwidth = 400;
    var graphheight = 250;
    var nextState = cw_storeGraphScores(
      lastState, scores, generationSize
    );
    console.log(scores, nextState);
    cw_clearGraphics(graphcanvas, graphctx, graphwidth, graphheight);
    cw_plotAverage(nextState, graphctx);
    cw_plotElite(nextState, graphctx);
    cw_plotTop(nextState, graphctx);
    cw_listTopScores(topScoresElem, nextState);
    nextState.scatterGraph = drawAllResults(
      scatterPlotElem, config, nextState, lastState.scatterGraph
    );
    return nextState;
  },
  clearGraphics: function(graphElem) {
    var graphcanvas = graphElem;
    var graphctx = graphcanvas.getContext("2d");
    var graphwidth = 400;
    var graphheight = 250;
    cw_clearGraphics(graphcanvas, graphctx, graphwidth, graphheight);
  }
};


function cw_storeGraphScores(lastState, cw_carScores, generationSize) {
  console.log(cw_carScores);
  return {
    cw_topScores: (lastState.cw_topScores || [])
    .concat([cw_carScores[0].score]),
    cw_graphAverage: (lastState.cw_graphAverage || []).concat([
      cw_average(cw_carScores, generationSize)
    ]),
    cw_graphElite: (lastState.cw_graphElite || []).concat([
      cw_eliteaverage(cw_carScores, generationSize)
    ]),
    cw_graphTop: (lastState.cw_graphTop || []).concat([
      cw_carScores[0].score.v
    ]),
    allResults: (lastState.allResults || []).concat(cw_carScores),
  }
}

function cw_plotTop(state, graphctx) {
  var cw_graphTop = state.cw_graphTop;
  var graphsize = cw_graphTop.length;
  graphctx.strokeStyle = "#C83B3B";
  graphctx.beginPath();
  graphctx.moveTo(0, 0);
  for (var k = 0; k < graphsize; k++) {
    graphctx.lineTo(400 * (k + 1) / graphsize, cw_graphTop[k]);
  }
  graphctx.stroke();
}

function cw_plotElite(state, graphctx) {
  var cw_graphElite = state.cw_graphElite;
  var graphsize = cw_graphElite.length;
  graphctx.strokeStyle = "#7BC74D";
  graphctx.beginPath();
  graphctx.moveTo(0, 0);
  for (var k = 0; k < graphsize; k++) {
    graphctx.lineTo(400 * (k + 1) / graphsize, cw_graphElite[k]);
  }
  graphctx.stroke();
}

function cw_plotAverage(state, graphctx) {
  var cw_graphAverage = state.cw_graphAverage;
  var graphsize = cw_graphAverage.length;
  graphctx.strokeStyle = "#3F72AF";
  graphctx.beginPath();
  graphctx.moveTo(0, 0);
  for (var k = 0; k < graphsize; k++) {
    graphctx.lineTo(400 * (k + 1) / graphsize, cw_graphAverage[k]);
  }
  graphctx.stroke();
}


function cw_eliteaverage(scores, generationSize) {
  var sum = 0;
  for (var k = 0; k < Math.floor(generationSize / 2); k++) {
    sum += scores[k].score.v;
  }
  return sum / Math.floor(generationSize / 2);
}

function cw_average(scores, generationSize) {
  var sum = 0;
  for (var k = 0; k < generationSize; k++) {
    sum += scores[k].score.v;
  }
  return sum / generationSize;
}

function cw_clearGraphics(graphcanvas, graphctx, graphwidth, graphheight) {
  graphcanvas.width = graphcanvas.width;
  graphctx.translate(0, graphheight);
  graphctx.scale(1, -1);
  graphctx.lineWidth = 1;
  graphctx.strokeStyle = "#3F72AF";
  graphctx.beginPath();
  graphctx.moveTo(0, graphheight / 2);
  graphctx.lineTo(graphwidth, graphheight / 2);
  graphctx.moveTo(0, graphheight / 4);
  graphctx.lineTo(graphwidth, graphheight / 4);
  graphctx.moveTo(0, graphheight * 3 / 4);
  graphctx.lineTo(graphwidth, graphheight * 3 / 4);
  graphctx.stroke();
}

function cw_listTopScores(elem, state) {
  var cw_topScores = state.cw_topScores;
  var ts = elem;
  ts.innerHTML = "<b>Top Scores:</b><br />";
  cw_topScores.sort(function (a, b) {
    if (a.v > b.v) {
      return -1
    } else {
      return 1
    }
  });

  for (var k = 0; k < Math.min(10, cw_topScores.length); k++) {
    var topScore = cw_topScores[k];
    // console.log(topScore);
    var n = "#" + (k + 1) + ":";
    var score = Math.round(topScore.v * 100) / 100;
    var distance = "d:" + Math.round(topScore.x * 100) / 100;
    var yrange =  "h:" + Math.round(topScore.y2 * 100) / 100 + "/" + Math.round(topScore.y * 100) / 100 + "m";
    var gen = "(Gen " + cw_topScores[k].i + ")"

    ts.innerHTML +=  [n, score, distance, yrange, gen].join(" ") + "<br />";
  }
}

function drawAllResults(scatterPlotElem, config, allResults, previousGraph){
  if(!scatterPlotElem) return;
  return scatterPlot(scatterPlotElem, allResults, config.propertyMap, previousGraph)
}

},{"./scatter-plot":11}],11:[function(require,module,exports){
/* globals vis Highcharts */

// Called when the Visualization API is loaded.

module.exports = highCharts;
function highCharts(elem, scores){
  var keys = Object.keys(scores[0].def);
  keys = keys.reduce(function(curArray, key){
    var l = scores[0].def[key].length;
    var subArray = [];
    for(var i = 0; i < l; i++){
      subArray.push(key + "." + i);
    }
    return curArray.concat(subArray);
  }, []);
  function retrieveValue(obj, path){
    return path.split(".").reduce(function(curValue, key){
      return curValue[key];
    }, obj);
  }

  var dataObj = Object.keys(scores).reduce(function(kv, score){
    keys.forEach(function(key){
      kv[key].data.push([
        retrieveValue(score.def, key), score.score.v
      ])
    })
    return kv;
  }, keys.reduce(function(kv, key){
    kv[key] = {
      name: key,
      data: [],
    }
    return kv;
  }, {}))
  Highcharts.chart(elem.id, {
      chart: {
          type: 'scatter',
          zoomType: 'xy'
      },
      title: {
          text: 'Property Value to Score'
      },
      xAxis: {
          title: {
              enabled: true,
              text: 'Normalized'
          },
          startOnTick: true,
          endOnTick: true,
          showLastLabel: true
      },
      yAxis: {
          title: {
              text: 'Score'
          }
      },
      legend: {
          layout: 'vertical',
          align: 'left',
          verticalAlign: 'top',
          x: 100,
          y: 70,
          floating: true,
          backgroundColor: (Highcharts.theme && Highcharts.theme.legendBackgroundColor) || '#FFFFFF',
          borderWidth: 1
      },
      plotOptions: {
          scatter: {
              marker: {
                  radius: 5,
                  states: {
                      hover: {
                          enabled: true,
                          lineColor: 'rgb(100,100,100)'
                      }
                  }
              },
              states: {
                  hover: {
                      marker: {
                          enabled: false
                      }
                  }
              },
              tooltip: {
                  headerFormat: '<b>{series.name}</b><br>',
                  pointFormat: '{point.x}, {point.y}'
              }
          }
      },
      series: keys.map(function(key){
        return dataObj[key];
      })
  });
}

function visChart(elem, scores, propertyMap, graph) {

  // Create and populate a data table.
  var data = new vis.DataSet();
  scores.forEach(function(scoreInfo){
    data.add({
      x: getProperty(scoreInfo, propertyMap.x),
      y: getProperty(scoreInfo, propertyMap.x),
      z: getProperty(scoreInfo, propertyMap.z),
      style: getProperty(scoreInfo, propertyMap.z),
      // extra: def.ancestry
    });
  });

  function getProperty(info, key){
    if(key === "score"){
      return info.score.v
    } else {
      return info.def[key];
    }
  }

  // specify options
  var options = {
    width:  '600px',
    height: '600px',
    style: 'dot-size',
    showPerspective: true,
    showLegend: true,
    showGrid: true,
    showShadow: false,

    // Option tooltip can be true, false, or a function returning a string with HTML contents
    tooltip: function (point) {
      // parameter point contains properties x, y, z, and data
      // data is the original object passed to the point constructor
      return 'score: <b>' + point.z + '</b><br>'; // + point.data.extra;
    },

    // Tooltip default styling can be overridden
    tooltipStyle: {
      content: {
        background    : 'rgba(255, 255, 255, 0.7)',
        padding       : '10px',
        borderRadius  : '10px'
      },
      line: {
        borderLeft    : '1px dotted rgba(0, 0, 0, 0.5)'
      },
      dot: {
        border        : '5px solid rgba(0, 0, 0, 0.5)'
      }
    },

    keepAspectRatio: true,
    verticalRatio: 0.5
  };

  var camera = graph ? graph.getCameraPosition() : null;

  // create our graph
  var container = elem;
  graph = new vis.Graph3d(container, data, options);

  if (camera) graph.setCameraPosition(camera); // restore camera position
  return graph;
}

},{}],12:[function(require,module,exports){

module.exports = generateRandom;
function generateRandom(){
  return Math.random();
}

},{}],13:[function(require,module,exports){
// http://sunmingtao.blogspot.com/2016/11/inbreeding-coefficient.html
module.exports = getInbreedingCoefficient;

function getInbreedingCoefficient(child){
  var nameIndex = new Map();
  var flagged = new Set();
  var convergencePoints = new Set();
  createAncestryMap(child, []);

  var storedCoefficients = new Map();

  return Array.from(convergencePoints.values()).reduce(function(sum, point){
    var iCo = getCoefficient(point);
    return sum + iCo;
  }, 0);

  function createAncestryMap(initNode){
    var itemsInQueue = [{ node: initNode, path: [] }];
    do{
      var item = itemsInQueue.shift();
      var node = item.node;
      var path = item.path;
      if(processItem(node, path)){
        var nextPath = [ node.id ].concat(path);
        itemsInQueue = itemsInQueue.concat(node.ancestry.map(function(parent){
          return {
            node: parent,
            path: nextPath
          };
        }));
      }
    }while(itemsInQueue.length);


    function processItem(node, path){
      var newAncestor = !nameIndex.has(node.id);
      if(newAncestor){
        nameIndex.set(node.id, {
          parents: (node.ancestry || []).map(function(parent){
            return parent.id;
          }),
          id: node.id,
          children: [],
          convergences: [],
        });
      } else {

        flagged.add(node.id)
        nameIndex.get(node.id).children.forEach(function(childIdentifier){
          var offsets = findConvergence(childIdentifier.path, path);
          if(!offsets){
            return;
          }
          var childID = path[offsets[1]];
          convergencePoints.add(childID);
          nameIndex.get(childID).convergences.push({
            parent: node.id,
            offsets: offsets,
          });
        });
      }

      if(path.length){
        nameIndex.get(node.id).children.push({
          child: path[0],
          path: path
        });
      }

      if(!newAncestor){
        return;
      }
      if(!node.ancestry){
        return;
      }
      return true;
    }
  }

  function getCoefficient(id){
    if(storedCoefficients.has(id)){
      return storedCoefficients.get(id);
    }
    var node = nameIndex.get(id);
    var val = node.convergences.reduce(function(sum, point){
      return sum + Math.pow(1 / 2, point.offsets.reduce(function(sum, value){
        return sum + value;
      }, 1)) * (1 + getCoefficient(point.parent));
    }, 0);
    storedCoefficients.set(id, val);

    return val;

  }
  function findConvergence(listA, listB){
    var ci, cj, li, lj;
    outerloop:
    for(ci = 0, li = listA.length; ci < li; ci++){
      for(cj = 0, lj = listB.length; cj < lj; cj++){
        if(listA[ci] === listB[cj]){
          break outerloop;
        }
      }
    }
    if(ci === li){
      return false;
    }
    return [ci, cj];
  }
}

},{}],14:[function(require,module,exports){
var carConstruct = require("../car-schema/construct.js");

var carConstants = carConstruct.carConstants();

var schema = carConstruct.generateSchema(carConstants);
var pickParent = require("./pickParent");
var selectFromAllParents = require("./selectFromAllParents");
const constants = {
  generationSize: 20,
  schema: schema,
  championLength: 1,
  mutation_range: 1,
  gen_mutation: 0.05,
};
module.exports = function(){
  var currentChoices = new Map();
  return Object.assign(
    {},
    constants,
    {
      selectFromAllParents: selectFromAllParents,
      generateRandom: require("./generateRandom"),
      pickParent: pickParent.bind(void 0, currentChoices),
    }
  );
}
module.exports.constants = constants

},{"../car-schema/construct.js":2,"./generateRandom":12,"./pickParent":15,"./selectFromAllParents":16}],15:[function(require,module,exports){
var nAttributes = 15;
module.exports = pickParent;

function pickParent(currentChoices, chooseId, key /* , parents */){
  if(!currentChoices.has(chooseId)){
    currentChoices.set(chooseId, initializePick())
  }
  // console.log(chooseId);
  var state = currentChoices.get(chooseId);
  // console.log(state.curparent);
  state.i++
  if(["wheel_radius", "wheel_vertex", "wheel_density"].indexOf(key) > -1){
    state.curparent = cw_chooseParent(state);
    return state.curparent;
  }
  state.curparent = cw_chooseParent(state);
  return state.curparent;

  function cw_chooseParent(state) {
    var curparent = state.curparent;
    var attributeIndex = state.i;
    var swapPoint1 = state.swapPoint1
    var swapPoint2 = state.swapPoint2
    // console.log(swapPoint1, swapPoint2, attributeIndex)
    if ((swapPoint1 == attributeIndex) || (swapPoint2 == attributeIndex)) {
      return curparent == 1 ? 0 : 1
    }
    return curparent
  }

  function initializePick(){
    var curparent = 0;

    var swapPoint1 = Math.floor(Math.random() * (nAttributes));
    var swapPoint2 = swapPoint1;
    while (swapPoint2 == swapPoint1) {
      swapPoint2 = Math.floor(Math.random() * (nAttributes));
    }
    var i = 0;
    return {
      curparent: curparent,
      i: i,
      swapPoint1: swapPoint1,
      swapPoint2: swapPoint2
    }
  }
}

},{}],16:[function(require,module,exports){
var getInbreedingCoefficient = require("./inbreeding-coefficient");

module.exports = simpleSelect;

function simpleSelect(parents){
  var totalParents = parents.length
  var r = Math.random();
  if (r == 0)
    return 0;
  return Math.floor(-Math.log(r) * totalParents) % totalParents;
}

function selectFromAllParents(parents, parentList, previousParentIndex) {
  var previousParent = parents[previousParentIndex];
  var validParents = parents.filter(function(parent, i){
    if(previousParentIndex === i){
      return false;
    }
    if(!previousParent){
      return true;
    }
    var child = {
      id: Math.random().toString(32),
      ancestry: [previousParent, parent].map(function(p){
        return {
          id: p.def.id,
          ancestry: p.def.ancestry
        }
      })
    }
    var iCo = getInbreedingCoefficient(child);
    console.log("inbreeding coefficient", iCo)
    if(iCo > 0.25){
      return false;
    }
    return true;
  })
  if(validParents.length === 0){
    return Math.floor(Math.random() * parents.length)
  }
  var totalScore = validParents.reduce(function(sum, parent){
    return sum + parent.score.v;
  }, 0);
  var r = totalScore * Math.random();
  for(var i = 0; i < validParents.length; i++){
    var score = validParents[i].score.v;
    if(r > score){
      r = r - score;
    } else {
      break;
    }
  }
  return i;
}

},{"./inbreeding-coefficient":13}],17:[function(require,module,exports){

module.exports = function(car) {
  var out = {
    chassis: ghost_get_chassis(car.chassis),
    wheels: [],
    pos: {x: car.chassis.GetPosition().x, y: car.chassis.GetPosition().y}
  };

  for (var i = 0; i < car.wheels.length; i++) {
    out.wheels[i] = ghost_get_wheel(car.wheels[i]);
  }

  return out;
}

function ghost_get_chassis(c) {
  var gc = [];

  for (var f = c.GetFixtureList(); f; f = f.m_next) {
    var s = f.GetShape();

    var p = {
      vtx: [],
      num: 0
    }

    p.num = s.m_vertexCount;

    for (var i = 0; i < s.m_vertexCount; i++) {
      p.vtx.push(c.GetWorldPoint(s.m_vertices[i]));
    }

    gc.push(p);
  }

  return gc;
}

function ghost_get_wheel(w) {
  var gw = [];

  for (var f = w.GetFixtureList(); f; f = f.m_next) {
    var s = f.GetShape();

    var c = {
      pos: w.GetWorldPoint(s.m_p),
      rad: s.m_radius,
      ang: w.m_sweep.a
    }

    gw.push(c);
  }

  return gw;
}

},{}],18:[function(require,module,exports){

var ghost_get_frame = require("./car-to-ghost.js");

var enable_ghost = true;

module.exports = {
  ghost_create_replay: ghost_create_replay,
  ghost_create_ghost: ghost_create_ghost,
  ghost_pause: ghost_pause,
  ghost_resume: ghost_resume,
  ghost_get_position: ghost_get_position,
  ghost_compare_to_replay: ghost_compare_to_replay,
  ghost_move_frame: ghost_move_frame,
  ghost_add_replay_frame: ghost_add_replay_frame,
  ghost_draw_frame: ghost_draw_frame,
  ghost_reset_ghost: ghost_reset_ghost
}

function ghost_create_replay() {
  if (!enable_ghost)
    return null;

  return {
    num_frames: 0,
    frames: [],
  }
}

function ghost_create_ghost() {
  if (!enable_ghost)
    return null;

  return {
    replay: null,
    frame: 0,
    dist: -100
  }
}

function ghost_reset_ghost(ghost) {
  if (!enable_ghost)
    return;
  if (ghost == null)
    return;
  ghost.frame = 0;
}

function ghost_pause(ghost) {
  if (ghost != null)
    ghost.old_frame = ghost.frame;
  ghost_reset_ghost(ghost);
}

function ghost_resume(ghost) {
  if (ghost != null)
    ghost.frame = ghost.old_frame;
}

function ghost_get_position(ghost) {
  if (!enable_ghost)
    return;
  if (ghost == null)
    return;
  if (ghost.frame < 0)
    return;
  if (ghost.replay == null)
    return;
  var frame = ghost.replay.frames[ghost.frame];
  return frame.pos;
}

function ghost_compare_to_replay(replay, ghost, max) {
  if (!enable_ghost)
    return;
  if (ghost == null)
    return;
  if (replay == null)
    return;

  if (ghost.dist < max) {
    ghost.replay = replay;
    ghost.dist = max;
    ghost.frame = 0;
  }
}

function ghost_move_frame(ghost) {
  if (!enable_ghost)
    return;
  if (ghost == null)
    return;
  if (ghost.replay == null)
    return;
  ghost.frame++;
  if (ghost.frame >= ghost.replay.num_frames)
    ghost.frame = ghost.replay.num_frames - 1;
}

function ghost_add_replay_frame(replay, car) {
  if (!enable_ghost)
    return;
  if (replay == null)
    return;

  var frame = ghost_get_frame(car);
  replay.frames.push(frame);
  replay.num_frames++;
}

function ghost_draw_frame(ctx, ghost, camera) {
  var zoom = camera.zoom;
  if (!enable_ghost)
    return;
  if (ghost == null)
    return;
  if (ghost.frame < 0)
    return;
  if (ghost.replay == null)
    return;

  var frame = ghost.replay.frames[ghost.frame];

  // wheel style
  ctx.fillStyle = "#eee";
  ctx.strokeStyle = "#aaa";
  ctx.lineWidth = 1 / zoom;

  for (var i = 0; i < frame.wheels.length; i++) {
    for (var w in frame.wheels[i]) {
      ghost_draw_circle(ctx, frame.wheels[i][w].pos, frame.wheels[i][w].rad, frame.wheels[i][w].ang);
    }
  }

  // chassis style
  ctx.strokeStyle = "#aaa";
  ctx.fillStyle = "#eee";
  ctx.lineWidth = 1 / zoom;
  ctx.beginPath();
  for (var c in frame.chassis)
    ghost_draw_poly(ctx, frame.chassis[c].vtx, frame.chassis[c].num);
  ctx.fill();
  ctx.stroke();
}

function ghost_draw_poly(ctx, vtx, n_vtx) {
  ctx.moveTo(vtx[0].x, vtx[0].y);
  for (var i = 1; i < n_vtx; i++) {
    ctx.lineTo(vtx[i].x, vtx[i].y);
  }
  ctx.lineTo(vtx[0].x, vtx[0].y);
}

function ghost_draw_circle(ctx, center, radius, angle) {
  ctx.beginPath();
  ctx.arc(center.x, center.y, radius, 0, 2 * Math.PI, true);

  ctx.moveTo(center.x, center.y);
  ctx.lineTo(center.x + radius * Math.cos(angle), center.y + radius * Math.sin(angle));

  ctx.fill();
  ctx.stroke();
}

},{"./car-to-ghost.js":17}],19:[function(require,module,exports){
/* globals document performance localStorage alert confirm btoa HTMLDivElement */
/* globals b2Vec2 */
// Global Vars

var worldRun = require("./world/run.js");
var carConstruct = require("./car-schema/construct.js");

var manageRound = require("./machine-learning/genetic-algorithm/manage-round.js");

var ghost_fns = require("./ghost/index.js");

var drawCar = require("./draw/draw-car.js");
var graph_fns = require("./draw/plot-graphs.js");
var plot_graphs = graph_fns.plotGraphs;
var cw_clearGraphics = graph_fns.clearGraphics;
var cw_drawFloor = require("./draw/draw-floor.js");

var ghost_draw_frame = ghost_fns.ghost_draw_frame;
var ghost_create_ghost = ghost_fns.ghost_create_ghost;
var ghost_add_replay_frame = ghost_fns.ghost_add_replay_frame;
var ghost_compare_to_replay = ghost_fns.ghost_compare_to_replay;
var ghost_get_position = ghost_fns.ghost_get_position;
var ghost_move_frame = ghost_fns.ghost_move_frame;
var ghost_reset_ghost = ghost_fns.ghost_reset_ghost
var ghost_pause = ghost_fns.ghost_pause;
var ghost_resume = ghost_fns.ghost_resume;
var ghost_create_replay = ghost_fns.ghost_create_replay;

var cw_Car = require("./draw/draw-car-stats.js");
var ghost;
var carMap = new Map();

var doDraw = true;
var cw_paused = false;

var box2dfps = 60;
var screenfps = 60;
var skipTicks = Math.round(1000 / box2dfps);
var maxFrameSkip = skipTicks * 2;

var canvas = document.getElementById("mainbox");
var ctx = canvas.getContext("2d");

var camera = {
  speed: 0.05,
  pos: {
    x: 0, y: 0
  },
  target: -1,
  zoom: 70
}

var minimapcamera = document.getElementById("minimapcamera").style;
var minimapholder = document.querySelector("#minimapholder");

var minimapcanvas = document.getElementById("minimap");
var minimapctx = minimapcanvas.getContext("2d");
var minimapscale = 3;
var minimapfogdistance = 0;
var fogdistance = document.getElementById("minimapfog").style;


var carConstants = carConstruct.carConstants();


var max_car_health = box2dfps * 10;

var cw_ghostReplayInterval = null;

var distanceMeter = document.getElementById("distancemeter");
var heightMeter = document.getElementById("heightmeter");

var leaderPosition = {
  x: 0, y: 0
}

minimapcamera.width = 12 * minimapscale + "px";
minimapcamera.height = 6 * minimapscale + "px";


// ======= WORLD STATE ======
var generationConfig = require("./generation-config");


var world_def = {
  gravity: new b2Vec2(0.0, -9.81),
  doSleep: true,
  floorseed: btoa(Math.seedrandom()),
  tileDimensions: new b2Vec2(1.5, 0.15),
  maxFloorTiles: 200,
  mutable_floor: false,
  box2dfps: box2dfps,
  motorSpeed: 80,
  max_car_health: max_car_health,
  schema: generationConfig.constants.schema
}

var cw_deadCars;
var graphState = {
  cw_topScores: [],
  cw_graphAverage: [],
  cw_graphElite: [],
  cw_graphTop: [],
};

function resetGraphState(){
  graphState = {
    cw_topScores: [],
    cw_graphAverage: [],
    cw_graphElite: [],
    cw_graphTop: [],
  };
}



// ==========================

var generationState;

// ======== Activity State ====
var currentRunner;
var loops = 0;
var nextGameTick = (new Date).getTime();

function showDistance(distance, height) {
  distanceMeter.innerHTML = distance + " meters<br />";
  heightMeter.innerHTML = height + " meters";
  if (distance > minimapfogdistance) {
    fogdistance.width = 800 - Math.round(distance + 15) * minimapscale + "px";
    minimapfogdistance = distance;
  }
}



/* === END Car ============================================================= */
/* ========================================================================= */


/* ========================================================================= */
/* ==== Generation ========================================================= */

function cw_generationZero() {

  generationState = manageRound.generationZero(generationConfig());
}

function resetCarUI(){
  cw_deadCars = 0;
  leaderPosition = {
    x: 0, y: 0
  };
  document.getElementById("generation").innerHTML = generationState.counter.toString();
  document.getElementById("cars").innerHTML = "";
  document.getElementById("population").innerHTML = generationConfig.constants.generationSize.toString();
}

/* ==== END Genration ====================================================== */
/* ========================================================================= */

/* ========================================================================= */
/* ==== Drawing ============================================================ */

function cw_drawScreen() {
  var floorTiles = currentRunner.scene.floorTiles;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.save();
  cw_setCameraPosition();
  var camera_x = camera.pos.x;
  var camera_y = camera.pos.y;
  var zoom = camera.zoom;
  ctx.translate(200 - (camera_x * zoom), 200 + (camera_y * zoom));
  ctx.scale(zoom, -zoom);
  cw_drawFloor(ctx, camera, floorTiles);
  ghost_draw_frame(ctx, ghost, camera);
  cw_drawCars();
  ctx.restore();
}

function cw_minimapCamera(/* x, y*/) {
  var camera_x = camera.pos.x
  var camera_y = camera.pos.y
  minimapcamera.left = Math.round((2 + camera_x) * minimapscale) + "px";
  minimapcamera.top = Math.round((31 - camera_y) * minimapscale) + "px";
}

function cw_setCameraTarget(k) {
  camera.target = k;
}

function cw_setCameraPosition() {
  var cameraTargetPosition
  if (camera.target !== -1) {
    cameraTargetPosition = carMap.get(camera.target).getPosition();
  } else {
    cameraTargetPosition = leaderPosition;
  }
  var diff_y = camera.pos.y - cameraTargetPosition.y;
  var diff_x = camera.pos.x - cameraTargetPosition.x;
  camera.pos.y -= camera.speed * diff_y;
  camera.pos.x -= camera.speed * diff_x;
  cw_minimapCamera(camera.pos.x, camera.pos.y);
}

function cw_drawGhostReplay() {
  var floorTiles = currentRunner.scene.floorTiles;
  var carPosition = ghost_get_position(ghost);
  camera.pos.x = carPosition.x;
  camera.pos.y = carPosition.y;
  cw_minimapCamera(camera.pos.x, camera.pos.y);
  showDistance(
    Math.round(carPosition.x * 100) / 100,
    Math.round(carPosition.y * 100) / 100
  );
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.save();
  ctx.translate(
    200 - (carPosition.x * camera.zoom),
    200 + (carPosition.y * camera.zoom)
  );
  ctx.scale(camera.zoom, -camera.zoom);
  ghost_draw_frame(ctx, ghost);
  ghost_move_frame(ghost);
  cw_drawFloor(ctx, camera, floorTiles);
  ctx.restore();
}


function cw_drawCars() {
  var cw_carArray = Array.from(carMap.values());
  for (var k = (cw_carArray.length - 1); k >= 0; k--) {
    var myCar = cw_carArray[k];
    drawCar(carConstants, myCar, camera, ctx)
  }
}

function toggleDisplay() {
  canvas.width = canvas.width;
  if (doDraw) {
    doDraw = false;
    cw_stopSimulation();
    cw_runningInterval = setInterval(function () {
      var time = performance.now() + (1000 / screenfps);
      while (time > performance.now()) {
        simulationStep();
      }
    }, 1);
  } else {
    doDraw = true;
    clearInterval(cw_runningInterval);
    cw_startSimulation();
  }
}

function cw_drawMiniMap() {
  var floorTiles = currentRunner.scene.floorTiles;
  var last_tile = null;
  var tile_position = new b2Vec2(-5, 0);
  minimapfogdistance = 0;
  fogdistance.width = "800px";
  minimapcanvas.width = minimapcanvas.width;
  minimapctx.strokeStyle = "#3F72AF";
  minimapctx.beginPath();
  minimapctx.moveTo(0, 35 * minimapscale);
  for (var k = 0; k < floorTiles.length; k++) {
    last_tile = floorTiles[k];
    var last_fixture = last_tile.GetFixtureList();
    var last_world_coords = last_tile.GetWorldPoint(last_fixture.GetShape().m_vertices[3]);
    tile_position = last_world_coords;
    minimapctx.lineTo((tile_position.x + 5) * minimapscale, (-tile_position.y + 35) * minimapscale);
  }
  minimapctx.stroke();
}

/* ==== END Drawing ======================================================== */
/* ========================================================================= */
var uiListeners = {
  preCarStep: function(){
    ghost_move_frame(ghost);
  },
  carStep(car){
    updateCarUI(car);
  },
  carDeath(carInfo){

    var k = carInfo.index;

    var car = carInfo.car, score = carInfo.score;
    carMap.get(carInfo).kill(currentRunner, world_def);

    // refocus camera to leader on death
    if (camera.target == carInfo) {
      cw_setCameraTarget(-1);
    }
    // console.log(score);
    carMap.delete(carInfo);
    ghost_compare_to_replay(car.replay, ghost, score.v);
    score.i = generationState.counter;

    cw_deadCars++;
    var generationSize = generationConfig.constants.generationSize;
    document.getElementById("population").innerHTML = (generationSize - cw_deadCars).toString();

    // console.log(leaderPosition.leader, k)
    if (leaderPosition.leader == k) {
      // leader is dead, find new leader
      cw_findLeader();
    }
  },
  generationEnd(results){
    cleanupRound(results);
    return cw_newRound(results);
  }
}

function simulationStep() {  
  currentRunner.step();
  showDistance(
    Math.round(leaderPosition.x * 100) / 100,
    Math.round(leaderPosition.y * 100) / 100
  );
}

function gameLoop() {
  loops = 0;
  while (!cw_paused && (new Date).getTime() > nextGameTick && loops < maxFrameSkip) {   
    nextGameTick += skipTicks;
    loops++;
  }
  simulationStep();
  cw_drawScreen();

  if(!cw_paused) window.requestAnimationFrame(gameLoop);
}

function updateCarUI(carInfo){
  var k = carInfo.index;
  var car = carMap.get(carInfo);
  var position = car.getPosition();

  ghost_add_replay_frame(car.replay, car.car.car);
  car.minimapmarker.style.left = Math.round((position.x + 5) * minimapscale) + "px";
  car.healthBar.width = Math.round((car.car.state.health / max_car_health) * 100) + "%";
  if (position.x > leaderPosition.x) {
    leaderPosition = position;
    leaderPosition.leader = k;
    // console.log("new leader: ", k);
  }
}

function cw_findLeader() {
  var lead = 0;
  var cw_carArray = Array.from(carMap.values());
  for (var k = 0; k < cw_carArray.length; k++) {
    if (!cw_carArray[k].alive) {
      continue;
    }
    var position = cw_carArray[k].getPosition();
    if (position.x > lead) {
      leaderPosition = position;
      leaderPosition.leader = k;
    }
  }
}

function fastForward(){
  var gen = generationState.counter;
  while(gen === generationState.counter){
    currentRunner.step();
  }
}

function cleanupRound(results){

  results.sort(function (a, b) {
    if (a.score.v > b.score.v) {
      return -1
    } else {
      return 1
    }
  })
  graphState = plot_graphs(
    document.getElementById("graphcanvas"),
    document.getElementById("topscores"),
    null,
    graphState,
    results
  );
}

function cw_newRound(results) {
  camera.pos.x = camera.pos.y = 0;
  cw_setCameraTarget(-1);

  generationState = manageRound.nextGeneration(
    generationState, results, generationConfig()
  );
  if (world_def.mutable_floor) {
    // GHOST DISABLED
    ghost = null;
    world_def.floorseed = btoa(Math.seedrandom());
  } else {
    // RE-ENABLE GHOST
    ghost_reset_ghost(ghost);
  }
  currentRunner = worldRun(world_def, generationState.generation, uiListeners);
  setupCarUI();
  cw_drawMiniMap();
  resetCarUI();
}

function cw_startSimulation() {
  cw_paused = false;
  window.requestAnimationFrame(gameLoop);
}

function cw_stopSimulation() {
  cw_paused = true;
}

function cw_clearPopulationWorld() {
  carMap.forEach(function(car){
    car.kill(currentRunner, world_def);
  });
}

function cw_resetPopulationUI() {
  document.getElementById("generation").innerHTML = "";
  document.getElementById("cars").innerHTML = "";
  document.getElementById("topscores").innerHTML = "";
  cw_clearGraphics(document.getElementById("graphcanvas"));
  resetGraphState();
}

function cw_resetWorld() {
  doDraw = true;
  cw_stopSimulation();
  world_def.floorseed = document.getElementById("newseed").value;
  cw_clearPopulationWorld();
  cw_resetPopulationUI();

  Math.seedrandom();
  cw_generationZero();
  currentRunner = worldRun(
    world_def, generationState.generation, uiListeners
  );

  ghost = ghost_create_ghost();
  resetCarUI();
  setupCarUI()
  cw_drawMiniMap();

  cw_startSimulation();
}

function setupCarUI(){
  currentRunner.cars.map(function(carInfo){
    var car = new cw_Car(carInfo, carMap);
    carMap.set(carInfo, car);
    car.replay = ghost_create_replay();
    ghost_add_replay_frame(car.replay, car.car.car);
  })
}


document.querySelector("#fast-forward").addEventListener("click", function(){
  fastForward()
});

document.querySelector("#save-progress").addEventListener("click", function(){
  saveProgress()
});

document.querySelector("#restore-progress").addEventListener("click", function(){
  restoreProgress()
});

document.querySelector("#toggle-display").addEventListener("click", function(){
  toggleDisplay()
})

document.querySelector("#new-population").addEventListener("click", function(){
  cw_resetPopulationUI()
  cw_generationZero();
  ghost = ghost_create_ghost();
  resetCarUI();
})

function saveProgress() {
  localStorage.cw_savedGeneration = JSON.stringify(generationState.generation);
  localStorage.cw_genCounter = generationState.counter;
  localStorage.cw_ghost = JSON.stringify(ghost);
  localStorage.cw_topScores = JSON.stringify(graphState.cw_topScores);
  localStorage.cw_floorSeed = world_def.floorseed;
}

function restoreProgress() {
  if (typeof localStorage.cw_savedGeneration == 'undefined' || localStorage.cw_savedGeneration == null) {
    alert("No saved progress found");
    return;
  }
  cw_stopSimulation();
  generationState.generation = JSON.parse(localStorage.cw_savedGeneration);
  generationState.counter = localStorage.cw_genCounter;
  ghost = JSON.parse(localStorage.cw_ghost);
  graphState.cw_topScores = JSON.parse(localStorage.cw_topScores);
  world_def.floorseed = localStorage.cw_floorSeed;
  document.getElementById("newseed").value = world_def.floorseed;

  currentRunner = worldRun(world_def, generationState.generation, uiListeners);
  cw_drawMiniMap();
  Math.seedrandom();

  resetCarUI();
  cw_startSimulation();
}

document.querySelector("#confirm-reset").addEventListener("click", function(){
  cw_confirmResetWorld()
})

function cw_confirmResetWorld() {
  if (confirm('Really reset world?')) {
    cw_resetWorld();
  } else {
    return false;
  }
}

// ghost replay stuff


function cw_pauseSimulation() {
  cw_paused = true;
  ghost_pause(ghost);
}

function cw_resumeSimulation() {
  cw_paused = false;
  ghost_resume(ghost);
  window.requestAnimationFrame(gameLoop);
}

function cw_startGhostReplay() {
  if (!doDraw) {
    toggleDisplay();
  }
  cw_pauseSimulation();
  cw_ghostReplayInterval = setInterval(cw_drawGhostReplay, Math.round(1000 / screenfps));
}

function cw_stopGhostReplay() {
  clearInterval(cw_ghostReplayInterval);
  cw_ghostReplayInterval = null;
  cw_findLeader();
  camera.pos.x = leaderPosition.x;
  camera.pos.y = leaderPosition.y;
  cw_resumeSimulation();
}

document.querySelector("#toggle-ghost").addEventListener("click", function(e){
  cw_toggleGhostReplay(e.target)
})

function cw_toggleGhostReplay(button) {
  if (cw_ghostReplayInterval == null) {
    cw_startGhostReplay();
    button.value = "Resume simulation";
  } else {
    cw_stopGhostReplay();
    button.value = "View top replay";
  }
}
// ghost replay stuff END

// initial stuff, only called once (hopefully)
function cw_init() {
  // clone silver dot and health bar
  var mmm = document.getElementsByName('minimapmarker')[0];
  var hbar = document.getElementsByName('healthbar')[0];
  var generationSize = generationConfig.constants.generationSize;

  for (var k = 0; k < generationSize; k++) {

    // minimap markers
    var newbar = mmm.cloneNode(true);
    newbar.id = "bar" + k;
    newbar.style.paddingTop = k * 9 + "px";
    minimapholder.appendChild(newbar);

    // health bars
    var newhealth = hbar.cloneNode(true);
    newhealth.getElementsByTagName("DIV")[0].id = "health" + k;
    newhealth.car_index = k;
    document.getElementById("health").appendChild(newhealth);
  }
  mmm.parentNode.removeChild(mmm);
  hbar.parentNode.removeChild(hbar);
  world_def.floorseed = btoa(Math.seedrandom());
  cw_generationZero();
  ghost = ghost_create_ghost();
  resetCarUI();
  currentRunner = worldRun(world_def, generationState.generation, uiListeners);
  setupCarUI();
  cw_drawMiniMap();
  window.requestAnimationFrame(gameLoop);
  
}

function relMouseCoords(event) {
  var totalOffsetX = 0;
  var totalOffsetY = 0;
  var canvasX = 0;
  var canvasY = 0;
  var currentElement = this;

  do {
    totalOffsetX += currentElement.offsetLeft - currentElement.scrollLeft;
    totalOffsetY += currentElement.offsetTop - currentElement.scrollTop;
    currentElement = currentElement.offsetParent
  }
  while (currentElement);

  canvasX = event.pageX - totalOffsetX;
  canvasY = event.pageY - totalOffsetY;

  return {x: canvasX, y: canvasY}
}
HTMLDivElement.prototype.relMouseCoords = relMouseCoords;
minimapholder.onclick = function (event) {
  var coords = minimapholder.relMouseCoords(event);
  var cw_carArray = Array.from(carMap.values());
  var closest = {
    value: cw_carArray[0].car,
    dist: Math.abs(((cw_carArray[0].getPosition().x + 6) * minimapscale) - coords.x),
    x: cw_carArray[0].getPosition().x
  }

  var maxX = 0;
  for (var i = 0; i < cw_carArray.length; i++) {
    var pos = cw_carArray[i].getPosition();
    var dist = Math.abs(((pos.x + 6) * minimapscale) - coords.x);
    if (dist < closest.dist) {
      closest.value = cw_carArray.car;
      closest.dist = dist;
      closest.x = pos.x;
    }
    maxX = Math.max(pos.x, maxX);
  }

  if (closest.x == maxX) { // focus on leader again
    cw_setCameraTarget(-1);
  } else {
    cw_setCameraTarget(closest.value);
  }
}


document.querySelector("#mutationrate").addEventListener("change", function(e){
  var elem = e.target
  cw_setMutation(elem.options[elem.selectedIndex].value)
})

document.querySelector("#mutationsize").addEventListener("change", function(e){
  var elem = e.target
  cw_setMutationRange(elem.options[elem.selectedIndex].value)
})

document.querySelector("#floor").addEventListener("change", function(e){
  var elem = e.target
  cw_setMutableFloor(elem.options[elem.selectedIndex].value)
});

document.querySelector("#gravity").addEventListener("change", function(e){
  var elem = e.target
  cw_setGravity(elem.options[elem.selectedIndex].value)
})

document.querySelector("#elitesize").addEventListener("change", function(e){
  var elem = e.target
  cw_setEliteSize(elem.options[elem.selectedIndex].value)
})

function cw_setMutation(mutation) {
  generationConfig.constants.gen_mutation = parseFloat(mutation);
}

function cw_setMutationRange(range) {
  generationConfig.constants.mutation_range = parseFloat(range);
}

function cw_setMutableFloor(choice) {
  world_def.mutable_floor = (choice == 1);
}

function cw_setGravity(choice) {
  world_def.gravity = new b2Vec2(0.0, -parseFloat(choice));
  var world = currentRunner.scene.world
  // CHECK GRAVITY CHANGES
  if (world.GetGravity().y != world_def.gravity.y) {
    world.SetGravity(world_def.gravity);
  }
}

function cw_setEliteSize(clones) {
  generationConfig.constants.championLength = parseInt(clones, 10);
}

cw_init();

},{"./car-schema/construct.js":2,"./draw/draw-car-stats.js":5,"./draw/draw-car.js":6,"./draw/draw-floor.js":8,"./draw/plot-graphs.js":10,"./generation-config":14,"./ghost/index.js":18,"./machine-learning/genetic-algorithm/manage-round.js":21,"./world/run.js":23}],20:[function(require,module,exports){
var random = require("./random.js");

module.exports = {
  createGenerationZero(schema, generator){
    return Object.keys(schema).reduce(function(instance, key){
      var schemaProp = schema[key];
      var values = random.createNormals(schemaProp, generator);
      instance[key] = values;
      return instance;
    }, { id: Math.random().toString(32) });
  },
  createCrossBreed(schema, parents, parentChooser){
    var id = Math.random().toString(32);
    return Object.keys(schema).reduce(function(crossDef, key){
      var schemaDef = schema[key];
      var values = [];
      for(var i = 0, l = schemaDef.length; i < l; i++){
        var p = parentChooser(id, key, parents);
        values.push(parents[p][key][i]);
      }
      crossDef[key] = values;
      return crossDef;
    }, {
      id: id,
      ancestry: parents.map(function(parent){
        return {
          id: parent.id,
          ancestry: parent.ancestry,
        };
      })
    });
  },
  createMutatedClone(schema, generator, parent, factor, chanceToMutate){
    return Object.keys(schema).reduce(function(clone, key){
      var schemaProp = schema[key];
      var originalValues = parent[key];
      var values = random.mutateNormals(
        schemaProp, generator, originalValues, factor, chanceToMutate
      );
      clone[key] = values;
      return clone;
    }, {
      id: parent.id,
      ancestry: parent.ancestry
    });
  },
  applyTypes(schema, parent){
    return Object.keys(schema).reduce(function(clone, key){
      var schemaProp = schema[key];
      var originalValues = parent[key];
      var values;
      switch(schemaProp.type){
        case "shuffle" :
          values = random.mapToShuffle(schemaProp, originalValues); break;
        case "float" :
          values = random.mapToFloat(schemaProp, originalValues); break;
        case "integer":
          values = random.mapToInteger(schemaProp, originalValues); break;
        default:
          throw new Error(`Unknown type ${schemaProp.type} of schema for key ${key}`);
      }
      clone[key] = values;
      return clone;
    }, {
      id: parent.id,
      ancestry: parent.ancestry
    });
  },
}

},{"./random.js":22}],21:[function(require,module,exports){
var create = require("../create-instance");

module.exports = {
  generationZero: generationZero,
  nextGeneration: nextGeneration
}

function generationZero(config){
  var generationSize = config.generationSize,
  schema = config.schema;
  var cw_carGeneration = [];
  for (var k = 0; k < generationSize; k++) {
    var def = create.createGenerationZero(schema, function(){
      return Math.random()
    });
    def.index = k;
    cw_carGeneration.push(def);
  }
  return {
    counter: 0,
    generation: cw_carGeneration,
  };
}

function nextGeneration(
  previousState,
  scores,
  config
){
  var champion_length = config.championLength,
    generationSize = config.generationSize,
    selectFromAllParents = config.selectFromAllParents;

  var newGeneration = new Array();
  var newborn;
  for (var k = 0; k < champion_length; k++) {``
    scores[k].def.is_elite = true;
    scores[k].def.index = k;
    newGeneration.push(scores[k].def);
  }
  var parentList = [];
  for (k = champion_length; k < generationSize; k++) {
    var parent1 = selectFromAllParents(scores, parentList);
    var parent2 = parent1;
    while (parent2 == parent1) {
      parent2 = selectFromAllParents(scores, parentList, parent1);
    }
    var pair = [parent1, parent2]
    parentList.push(pair);
    newborn = makeChild(config,
      pair.map(function(parent) { return scores[parent].def; })
    );
    newborn = mutate(config, newborn);
    newborn.is_elite = false;
    newborn.index = k;
    newGeneration.push(newborn);
  }

  return {
    counter: previousState.counter + 1,
    generation: newGeneration,
  };
}


function makeChild(config, parents){
  var schema = config.schema,
    pickParent = config.pickParent;
  return create.createCrossBreed(schema, parents, pickParent)
}


function mutate(config, parent){
  var schema = config.schema,
    mutation_range = config.mutation_range,
    gen_mutation = config.gen_mutation,
    generateRandom = config.generateRandom;
  return create.createMutatedClone(
    schema,
    generateRandom,
    parent,
    Math.max(mutation_range),
    gen_mutation
  )
}

},{"../create-instance":20}],22:[function(require,module,exports){


const random = {
  shuffleIntegers(prop, generator){
    return random.mapToShuffle(prop, random.createNormals({
      length: prop.length || 10,
      inclusive: true,
    }, generator));
  },
  createIntegers(prop, generator){
    return random.mapToInteger(prop, random.createNormals({
      length: prop.length,
      inclusive: true,
    }, generator));
  },
  createFloats(prop, generator){
    return random.mapToFloat(prop, random.createNormals({
      length: prop.length,
      inclusive: true,
    }, generator));
  },
  createNormals(prop, generator){
    var l = prop.length;
    var values = [];
    for(var i = 0; i < l; i++){
      values.push(
        createNormal(prop, generator)
      );
    }
    return values;
  },
  mutateShuffle(
    prop, generator, originalValues, mutation_range, chanceToMutate
  ){
    return random.mapToShuffle(prop, random.mutateNormals(
      prop, generator, originalValues, mutation_range, chanceToMutate
    ));
  },
  mutateIntegers(prop, generator, originalValues, mutation_range, chanceToMutate){
    return random.mapToInteger(prop, random.mutateNormals(
      prop, generator, originalValues, mutation_range, chanceToMutate
    ));
  },
  mutateFloats(prop, generator, originalValues, mutation_range, chanceToMutate){
    return random.mapToFloat(prop, random.mutateNormals(
      prop, generator, originalValues, mutation_range, chanceToMutate
    ));
  },
  mapToShuffle(prop, normals){
    var offset = prop.offset || 0;
    var limit = prop.limit || prop.length;
    var sorted = normals.slice().sort(function(a, b){
      return a - b;
    });
    return normals.map(function(val){
      return sorted.indexOf(val);
    }).map(function(i){
      return i + offset;
    }).slice(0, limit);
  },
  mapToInteger(prop, normals){
    prop = {
      min: prop.min || 0,
      range: prop.range || 10,
      length: prop.length
    }
    return random.mapToFloat(prop, normals).map(function(float){
      return Math.round(float);
    });
  },
  mapToFloat(prop, normals){
    prop = {
      min: prop.min || 0,
      range: prop.range || 1
    }
    return normals.map(function(normal){
      var min = prop.min;
      var range = prop.range;
      return min + normal * range
    })
  },
  mutateNormals(prop, generator, originalValues, mutation_range, chanceToMutate){
    var factor = (prop.factor || 1) * mutation_range
    return originalValues.map(function(originalValue){
      if(generator() > chanceToMutate){
        return originalValue;
      }
      return mutateNormal(
        prop, generator, originalValue, factor
      );
    });
  }
};

module.exports = random;

function mutateNormal(prop, generator, originalValue, mutation_range){
  if(mutation_range > 1){
    throw new Error("Cannot mutate beyond bounds");
  }
  var newMin = originalValue - 0.5;
  if (newMin < 0) newMin = 0;
  if (newMin + mutation_range  > 1)
    newMin = 1 - mutation_range;
  var rangeValue = createNormal({
    inclusive: true,
  }, generator);
  return newMin + rangeValue * mutation_range;
}

function createNormal(prop, generator){
  if(!prop.inclusive){
    return generator();
  } else {
    return generator() < 0.5 ?
    generator() :
    1 - generator();
  }
}

},{}],23:[function(require,module,exports){
/* globals btoa */
var setupScene = require("./setup-scene");
var carRun = require("../car-schema/run");
var defToCar = require("../car-schema/def-to-car");

module.exports = runDefs;
function runDefs(world_def, defs, listeners) {
  if (world_def.mutable_floor) {
    // GHOST DISABLED
    world_def.floorseed = btoa(Math.seedrandom());
  }

  var scene = setupScene(world_def);
  scene.world.Step(1 / world_def.box2dfps, 20, 20);
  console.log("about to build cars");
  var cars = defs.map((def, i) => {
    return {
      index: i,
      def: def,
      car: defToCar(def, scene.world, world_def),
      state: carRun.getInitialState(world_def)
    };
  });
  var alivecars = cars;
  return {
    scene: scene,
    cars: cars,
    step: function () {
      if (alivecars.length === 0) {
        throw new Error("no more cars");
      }
      scene.world.Step(1 / world_def.box2dfps, 20, 20);
      listeners.preCarStep();
      alivecars = alivecars.filter(function (car) {
        car.state = carRun.updateState(
          world_def, car.car, car.state
        );
        var status = carRun.getStatus(car.state, world_def);
        listeners.carStep(car);
        if (status === 0) {
          return true;
        }
        car.score = carRun.calculateScore(car.state, world_def);
        listeners.carDeath(car);

        var world = scene.world;
        var worldCar = car.car;
        world.DestroyBody(worldCar.chassis);

        for (var w = 0; w < worldCar.wheels.length; w++) {
          world.DestroyBody(worldCar.wheels[w]);
        }

        return false;
      })
      if (alivecars.length === 0) {
        listeners.generationEnd(cars);
      }
    }
  }

}

},{"../car-schema/def-to-car":3,"../car-schema/run":4,"./setup-scene":24}],24:[function(require,module,exports){
/* globals b2World b2Vec2 b2BodyDef b2FixtureDef b2PolygonShape */

/*

world_def = {
  gravity: {x, y},
  doSleep: boolean,
  floorseed: string,
  tileDimensions,
  maxFloorTiles,
  mutable_floor: boolean
}

*/

module.exports = function(world_def){

  var world = new b2World(world_def.gravity, world_def.doSleep);
  var floorTiles = cw_createFloor(
    world,
    world_def.floorseed,
    world_def.tileDimensions,
    world_def.maxFloorTiles,
    world_def.mutable_floor
  );

  var last_tile = floorTiles[
    floorTiles.length - 1
  ];
  var last_fixture = last_tile.GetFixtureList();
  var tile_position = last_tile.GetWorldPoint(
    last_fixture.GetShape().m_vertices[3]
  );
  world.finishLine = tile_position.x;
  return {
    world: world,
    floorTiles: floorTiles,
    finishLine: tile_position.x
  };
}

function cw_createFloor(world, floorseed, dimensions, maxFloorTiles, mutable_floor) {
  var last_tile = null;
  var tile_position = new b2Vec2(-5, 0);
  var cw_floorTiles = [];
  Math.seedrandom(floorseed);
  for (var k = 0; k < maxFloorTiles; k++) {
    if (!mutable_floor) {
      // keep old impossible tracks if not using mutable floors
      last_tile = cw_createFloorTile(
        world, dimensions, tile_position, (Math.random() * 3 - 1.5) * 1.5 * k / maxFloorTiles
      );
    } else {
      // if path is mutable over races, create smoother tracks
      last_tile = cw_createFloorTile(
        world, dimensions, tile_position, (Math.random() * 3 - 1.5) * 1.2 * k / maxFloorTiles
      );
    }
    cw_floorTiles.push(last_tile);
    var last_fixture = last_tile.GetFixtureList();
    tile_position = last_tile.GetWorldPoint(last_fixture.GetShape().m_vertices[3]);
  }
  return cw_floorTiles;
}


function cw_createFloorTile(world, dim, position, angle) {
  var body_def = new b2BodyDef();

  body_def.position.Set(position.x, position.y);
  var body = world.CreateBody(body_def);
  var fix_def = new b2FixtureDef();
  fix_def.shape = new b2PolygonShape();
  fix_def.friction = 0.5;

  var coords = new Array();
  coords.push(new b2Vec2(0, 0));
  coords.push(new b2Vec2(0, -dim.y));
  coords.push(new b2Vec2(dim.x, -dim.y));
  coords.push(new b2Vec2(dim.x, 0));

  var center = new b2Vec2(0, 0);

  var newcoords = cw_rotateFloorTile(coords, center, angle);

  fix_def.shape.SetAsArray(newcoords);

  body.CreateFixture(fix_def);
  return body;
}

function cw_rotateFloorTile(coords, center, angle) {
  return coords.map(function(coord){
    return {
      x: Math.cos(angle) * (coord.x - center.x) - Math.sin(angle) * (coord.y - center.y) + center.x,
      y: Math.sin(angle) * (coord.x - center.x) + Math.cos(angle) * (coord.y - center.y) + center.y,
    };
  });
}

},{}]},{},[19])
//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIi4uLy4uLy4uL0FwcERhdGEvUm9hbWluZy9ucG0vbm9kZV9tb2R1bGVzL2Jyb3dzZXJpZnkvbm9kZV9tb2R1bGVzL2Jyb3dzZXItcGFjay9fcHJlbHVkZS5qcyIsInNyYy9jYXItc2NoZW1hL2Nhci1jb25zdGFudHMuanNvbiIsInNyYy9jYXItc2NoZW1hL2NvbnN0cnVjdC5qcyIsInNyYy9jYXItc2NoZW1hL2RlZi10by1jYXIuanMiLCJzcmMvY2FyLXNjaGVtYS9ydW4uanMiLCJzcmMvZHJhdy9kcmF3LWNhci1zdGF0cy5qcyIsInNyYy9kcmF3L2RyYXctY2FyLmpzIiwic3JjL2RyYXcvZHJhdy1jaXJjbGUuanMiLCJzcmMvZHJhdy9kcmF3LWZsb29yLmpzIiwic3JjL2RyYXcvZHJhdy12aXJ0dWFsLXBvbHkuanMiLCJzcmMvZHJhdy9wbG90LWdyYXBocy5qcyIsInNyYy9kcmF3L3NjYXR0ZXItcGxvdC5qcyIsInNyYy9nZW5lcmF0aW9uLWNvbmZpZy9nZW5lcmF0ZVJhbmRvbS5qcyIsInNyYy9nZW5lcmF0aW9uLWNvbmZpZy9pbmJyZWVkaW5nLWNvZWZmaWNpZW50LmpzIiwic3JjL2dlbmVyYXRpb24tY29uZmlnL2luZGV4LmpzIiwic3JjL2dlbmVyYXRpb24tY29uZmlnL3BpY2tQYXJlbnQuanMiLCJzcmMvZ2VuZXJhdGlvbi1jb25maWcvc2VsZWN0RnJvbUFsbFBhcmVudHMuanMiLCJzcmMvZ2hvc3QvY2FyLXRvLWdob3N0LmpzIiwic3JjL2dob3N0L2luZGV4LmpzIiwic3JjL2luZGV4LmpzIiwic3JjL21hY2hpbmUtbGVhcm5pbmcvY3JlYXRlLWluc3RhbmNlLmpzIiwic3JjL21hY2hpbmUtbGVhcm5pbmcvZ2VuZXRpYy1hbGdvcml0aG0vbWFuYWdlLXJvdW5kLmpzIiwic3JjL21hY2hpbmUtbGVhcm5pbmcvcmFuZG9tLmpzIiwic3JjL3dvcmxkL3J1bi5qcyIsInNyYy93b3JsZC9zZXR1cC1zY2VuZS5qcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTtBQ0FBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNYQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDcEVBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3JIQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUM5RUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDL0RBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3REQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ2hCQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDbkNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNkQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN0SkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3BLQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDTEE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQzlHQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUMzQkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQy9DQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN0REE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN2REE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDbEtBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN0c0JBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3JFQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3JGQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDdkhBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUM5REE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0EiLCJmaWxlIjoiZ2VuZXJhdGVkLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXNDb250ZW50IjpbIihmdW5jdGlvbigpe2Z1bmN0aW9uIHIoZSxuLHQpe2Z1bmN0aW9uIG8oaSxmKXtpZighbltpXSl7aWYoIWVbaV0pe3ZhciBjPVwiZnVuY3Rpb25cIj09dHlwZW9mIHJlcXVpcmUmJnJlcXVpcmU7aWYoIWYmJmMpcmV0dXJuIGMoaSwhMCk7aWYodSlyZXR1cm4gdShpLCEwKTt2YXIgYT1uZXcgRXJyb3IoXCJDYW5ub3QgZmluZCBtb2R1bGUgJ1wiK2krXCInXCIpO3Rocm93IGEuY29kZT1cIk1PRFVMRV9OT1RfRk9VTkRcIixhfXZhciBwPW5baV09e2V4cG9ydHM6e319O2VbaV1bMF0uY2FsbChwLmV4cG9ydHMsZnVuY3Rpb24ocil7dmFyIG49ZVtpXVsxXVtyXTtyZXR1cm4gbyhufHxyKX0scCxwLmV4cG9ydHMscixlLG4sdCl9cmV0dXJuIG5baV0uZXhwb3J0c31mb3IodmFyIHU9XCJmdW5jdGlvblwiPT10eXBlb2YgcmVxdWlyZSYmcmVxdWlyZSxpPTA7aTx0Lmxlbmd0aDtpKyspbyh0W2ldKTtyZXR1cm4gb31yZXR1cm4gcn0pKCkiLCJtb2R1bGUuZXhwb3J0cz17XHJcbiAgXCJ3aGVlbENvdW50XCI6IDIsXHJcbiAgXCJ3aGVlbE1pblJhZGl1c1wiOiAwLjIsXHJcbiAgXCJ3aGVlbFJhZGl1c1JhbmdlXCI6IDAuNSxcclxuICBcIndoZWVsTWluRGVuc2l0eVwiOiA0MCxcclxuICBcIndoZWVsRGVuc2l0eVJhbmdlXCI6IDEwMCxcclxuICBcImNoYXNzaXNEZW5zaXR5UmFuZ2VcIjogMzAwLFxyXG4gIFwiY2hhc3Npc01pbkRlbnNpdHlcIjogMzAsXHJcbiAgXCJjaGFzc2lzTWluQXhpc1wiOiAwLjEsXHJcbiAgXCJjaGFzc2lzQXhpc1JhbmdlXCI6IDEuMVxyXG59XHJcbiIsInZhciBjYXJDb25zdGFudHMgPSByZXF1aXJlKFwiLi9jYXItY29uc3RhbnRzLmpzb25cIik7XHJcblxyXG5tb2R1bGUuZXhwb3J0cyA9IHtcclxuICB3b3JsZERlZjogd29ybGREZWYsXHJcbiAgY2FyQ29uc3RhbnRzOiBnZXRDYXJDb25zdGFudHMsXHJcbiAgZ2VuZXJhdGVTY2hlbWE6IGdlbmVyYXRlU2NoZW1hXHJcbn1cclxuXHJcbmZ1bmN0aW9uIHdvcmxkRGVmKCl7XHJcbiAgdmFyIGJveDJkZnBzID0gNjA7XHJcbiAgcmV0dXJuIHtcclxuICAgIGdyYXZpdHk6IHsgeTogMCB9LFxyXG4gICAgZG9TbGVlcDogdHJ1ZSxcclxuICAgIGZsb29yc2VlZDogXCJhYmNcIixcclxuICAgIG1heEZsb29yVGlsZXM6IDIwMCxcclxuICAgIG11dGFibGVfZmxvb3I6IGZhbHNlLFxyXG4gICAgbW90b3JTcGVlZDogMjAsXHJcbiAgICBib3gyZGZwczogYm94MmRmcHMsXHJcbiAgICBtYXhfY2FyX2hlYWx0aDogYm94MmRmcHMgKiAxMCxcclxuICAgIHRpbGVEaW1lbnNpb25zOiB7XHJcbiAgICAgIHdpZHRoOiAxLjUsXHJcbiAgICAgIGhlaWdodDogMC4xNVxyXG4gICAgfVxyXG4gIH07XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGdldENhckNvbnN0YW50cygpe1xyXG4gIHJldHVybiBjYXJDb25zdGFudHM7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGdlbmVyYXRlU2NoZW1hKHZhbHVlcyl7XHJcbiAgcmV0dXJuIHtcclxuICAgIHdoZWVsX3JhZGl1czoge1xyXG4gICAgICB0eXBlOiBcImZsb2F0XCIsXHJcbiAgICAgIGxlbmd0aDogdmFsdWVzLndoZWVsQ291bnQsXHJcbiAgICAgIG1pbjogdmFsdWVzLndoZWVsTWluUmFkaXVzLFxyXG4gICAgICByYW5nZTogdmFsdWVzLndoZWVsUmFkaXVzUmFuZ2UsXHJcbiAgICAgIGZhY3RvcjogMSxcclxuICAgIH0sXHJcbiAgICB3aGVlbF9kZW5zaXR5OiB7XHJcbiAgICAgIHR5cGU6IFwiZmxvYXRcIixcclxuICAgICAgbGVuZ3RoOiB2YWx1ZXMud2hlZWxDb3VudCxcclxuICAgICAgbWluOiB2YWx1ZXMud2hlZWxNaW5EZW5zaXR5LFxyXG4gICAgICByYW5nZTogdmFsdWVzLndoZWVsRGVuc2l0eVJhbmdlLFxyXG4gICAgICBmYWN0b3I6IDEsXHJcbiAgICB9LFxyXG4gICAgY2hhc3Npc19kZW5zaXR5OiB7XHJcbiAgICAgIHR5cGU6IFwiZmxvYXRcIixcclxuICAgICAgbGVuZ3RoOiAxLFxyXG4gICAgICBtaW46IHZhbHVlcy5jaGFzc2lzRGVuc2l0eVJhbmdlLFxyXG4gICAgICByYW5nZTogdmFsdWVzLmNoYXNzaXNNaW5EZW5zaXR5LFxyXG4gICAgICBmYWN0b3I6IDEsXHJcbiAgICB9LFxyXG4gICAgdmVydGV4X2xpc3Q6IHtcclxuICAgICAgdHlwZTogXCJmbG9hdFwiLFxyXG4gICAgICBsZW5ndGg6IDEyLFxyXG4gICAgICBtaW46IHZhbHVlcy5jaGFzc2lzTWluQXhpcyxcclxuICAgICAgcmFuZ2U6IHZhbHVlcy5jaGFzc2lzQXhpc1JhbmdlLFxyXG4gICAgICBmYWN0b3I6IDEsXHJcbiAgICB9LFxyXG4gICAgd2hlZWxfdmVydGV4OiB7XHJcbiAgICAgIHR5cGU6IFwic2h1ZmZsZVwiLFxyXG4gICAgICBsZW5ndGg6IDgsXHJcbiAgICAgIGxpbWl0OiB2YWx1ZXMud2hlZWxDb3VudCxcclxuICAgICAgZmFjdG9yOiAxLFxyXG4gICAgfSxcclxuICB9O1xyXG59XHJcbiIsIi8qXHJcbiAgZ2xvYmFscyBiMlJldm9sdXRlSm9pbnREZWYgYjJWZWMyIGIyQm9keURlZiBiMkJvZHkgYjJGaXh0dXJlRGVmIGIyUG9seWdvblNoYXBlIGIyQ2lyY2xlU2hhcGVcclxuKi9cclxuXHJcbnZhciBjcmVhdGVJbnN0YW5jZSA9IHJlcXVpcmUoXCIuLi9tYWNoaW5lLWxlYXJuaW5nL2NyZWF0ZS1pbnN0YW5jZVwiKTtcclxuXHJcbm1vZHVsZS5leHBvcnRzID0gZGVmVG9DYXI7XHJcblxyXG5mdW5jdGlvbiBkZWZUb0Nhcihub3JtYWxfZGVmLCB3b3JsZCwgY29uc3RhbnRzKXtcclxuICB2YXIgY2FyX2RlZiA9IGNyZWF0ZUluc3RhbmNlLmFwcGx5VHlwZXMoY29uc3RhbnRzLnNjaGVtYSwgbm9ybWFsX2RlZilcclxuICB2YXIgaW5zdGFuY2UgPSB7fTtcclxuICBpbnN0YW5jZS5jaGFzc2lzID0gY3JlYXRlQ2hhc3NpcyhcclxuICAgIHdvcmxkLCBjYXJfZGVmLnZlcnRleF9saXN0LCBjYXJfZGVmLmNoYXNzaXNfZGVuc2l0eVxyXG4gICk7XHJcbiAgdmFyIGk7XHJcblxyXG4gIHZhciB3aGVlbENvdW50ID0gY2FyX2RlZi53aGVlbF9yYWRpdXMubGVuZ3RoO1xyXG5cclxuICBpbnN0YW5jZS53aGVlbHMgPSBbXTtcclxuICBmb3IgKGkgPSAwOyBpIDwgd2hlZWxDb3VudDsgaSsrKSB7XHJcbiAgICBpbnN0YW5jZS53aGVlbHNbaV0gPSBjcmVhdGVXaGVlbChcclxuICAgICAgd29ybGQsXHJcbiAgICAgIGNhcl9kZWYud2hlZWxfcmFkaXVzW2ldLFxyXG4gICAgICBjYXJfZGVmLndoZWVsX2RlbnNpdHlbaV1cclxuICAgICk7XHJcbiAgfVxyXG5cclxuICB2YXIgY2FybWFzcyA9IGluc3RhbmNlLmNoYXNzaXMuR2V0TWFzcygpO1xyXG4gIGZvciAoaSA9IDA7IGkgPCB3aGVlbENvdW50OyBpKyspIHtcclxuICAgIGNhcm1hc3MgKz0gaW5zdGFuY2Uud2hlZWxzW2ldLkdldE1hc3MoKTtcclxuICB9XHJcblxyXG4gIHZhciBqb2ludF9kZWYgPSBuZXcgYjJSZXZvbHV0ZUpvaW50RGVmKCk7XHJcblxyXG4gIGZvciAoaSA9IDA7IGkgPCB3aGVlbENvdW50OyBpKyspIHtcclxuICAgIHZhciB0b3JxdWUgPSBjYXJtYXNzICogLWNvbnN0YW50cy5ncmF2aXR5LnkgLyBjYXJfZGVmLndoZWVsX3JhZGl1c1tpXTtcclxuXHJcbiAgICB2YXIgcmFuZHZlcnRleCA9IGluc3RhbmNlLmNoYXNzaXMudmVydGV4X2xpc3RbY2FyX2RlZi53aGVlbF92ZXJ0ZXhbaV1dO1xyXG4gICAgam9pbnRfZGVmLmxvY2FsQW5jaG9yQS5TZXQocmFuZHZlcnRleC54LCByYW5kdmVydGV4LnkpO1xyXG4gICAgam9pbnRfZGVmLmxvY2FsQW5jaG9yQi5TZXQoMCwgMCk7XHJcbiAgICBqb2ludF9kZWYubWF4TW90b3JUb3JxdWUgPSB0b3JxdWU7XHJcbiAgICBqb2ludF9kZWYubW90b3JTcGVlZCA9IC1jb25zdGFudHMubW90b3JTcGVlZDtcclxuICAgIGpvaW50X2RlZi5lbmFibGVNb3RvciA9IHRydWU7XHJcbiAgICBqb2ludF9kZWYuYm9keUEgPSBpbnN0YW5jZS5jaGFzc2lzO1xyXG4gICAgam9pbnRfZGVmLmJvZHlCID0gaW5zdGFuY2Uud2hlZWxzW2ldO1xyXG4gICAgd29ybGQuQ3JlYXRlSm9pbnQoam9pbnRfZGVmKTtcclxuICB9XHJcblxyXG4gIHJldHVybiBpbnN0YW5jZTtcclxufVxyXG5cclxuZnVuY3Rpb24gY3JlYXRlQ2hhc3Npcyh3b3JsZCwgdmVydGV4cywgZGVuc2l0eSkge1xyXG5cclxuICB2YXIgdmVydGV4X2xpc3QgPSBuZXcgQXJyYXkoKTtcclxuICB2ZXJ0ZXhfbGlzdC5wdXNoKG5ldyBiMlZlYzIodmVydGV4c1swXSwgMCkpO1xyXG4gIHZlcnRleF9saXN0LnB1c2gobmV3IGIyVmVjMih2ZXJ0ZXhzWzFdLCB2ZXJ0ZXhzWzJdKSk7XHJcbiAgdmVydGV4X2xpc3QucHVzaChuZXcgYjJWZWMyKDAsIHZlcnRleHNbM10pKTtcclxuICB2ZXJ0ZXhfbGlzdC5wdXNoKG5ldyBiMlZlYzIoLXZlcnRleHNbNF0sIHZlcnRleHNbNV0pKTtcclxuICB2ZXJ0ZXhfbGlzdC5wdXNoKG5ldyBiMlZlYzIoLXZlcnRleHNbNl0sIDApKTtcclxuICB2ZXJ0ZXhfbGlzdC5wdXNoKG5ldyBiMlZlYzIoLXZlcnRleHNbN10sIC12ZXJ0ZXhzWzhdKSk7XHJcbiAgdmVydGV4X2xpc3QucHVzaChuZXcgYjJWZWMyKDAsIC12ZXJ0ZXhzWzldKSk7XHJcbiAgdmVydGV4X2xpc3QucHVzaChuZXcgYjJWZWMyKHZlcnRleHNbMTBdLCAtdmVydGV4c1sxMV0pKTtcclxuXHJcbiAgdmFyIGJvZHlfZGVmID0gbmV3IGIyQm9keURlZigpO1xyXG4gIGJvZHlfZGVmLnR5cGUgPSBiMkJvZHkuYjJfZHluYW1pY0JvZHk7XHJcbiAgYm9keV9kZWYucG9zaXRpb24uU2V0KDAuMCwgNC4wKTtcclxuXHJcbiAgdmFyIGJvZHkgPSB3b3JsZC5DcmVhdGVCb2R5KGJvZHlfZGVmKTtcclxuXHJcbiAgY3JlYXRlQ2hhc3Npc1BhcnQoYm9keSwgdmVydGV4X2xpc3RbMF0sIHZlcnRleF9saXN0WzFdLCBkZW5zaXR5KTtcclxuICBjcmVhdGVDaGFzc2lzUGFydChib2R5LCB2ZXJ0ZXhfbGlzdFsxXSwgdmVydGV4X2xpc3RbMl0sIGRlbnNpdHkpO1xyXG4gIGNyZWF0ZUNoYXNzaXNQYXJ0KGJvZHksIHZlcnRleF9saXN0WzJdLCB2ZXJ0ZXhfbGlzdFszXSwgZGVuc2l0eSk7XHJcbiAgY3JlYXRlQ2hhc3Npc1BhcnQoYm9keSwgdmVydGV4X2xpc3RbM10sIHZlcnRleF9saXN0WzRdLCBkZW5zaXR5KTtcclxuICBjcmVhdGVDaGFzc2lzUGFydChib2R5LCB2ZXJ0ZXhfbGlzdFs0XSwgdmVydGV4X2xpc3RbNV0sIGRlbnNpdHkpO1xyXG4gIGNyZWF0ZUNoYXNzaXNQYXJ0KGJvZHksIHZlcnRleF9saXN0WzVdLCB2ZXJ0ZXhfbGlzdFs2XSwgZGVuc2l0eSk7XHJcbiAgY3JlYXRlQ2hhc3Npc1BhcnQoYm9keSwgdmVydGV4X2xpc3RbNl0sIHZlcnRleF9saXN0WzddLCBkZW5zaXR5KTtcclxuICBjcmVhdGVDaGFzc2lzUGFydChib2R5LCB2ZXJ0ZXhfbGlzdFs3XSwgdmVydGV4X2xpc3RbMF0sIGRlbnNpdHkpO1xyXG5cclxuICBib2R5LnZlcnRleF9saXN0ID0gdmVydGV4X2xpc3Q7XHJcblxyXG4gIHJldHVybiBib2R5O1xyXG59XHJcblxyXG5cclxuZnVuY3Rpb24gY3JlYXRlQ2hhc3Npc1BhcnQoYm9keSwgdmVydGV4MSwgdmVydGV4MiwgZGVuc2l0eSkge1xyXG4gIHZhciB2ZXJ0ZXhfbGlzdCA9IG5ldyBBcnJheSgpO1xyXG4gIHZlcnRleF9saXN0LnB1c2godmVydGV4MSk7XHJcbiAgdmVydGV4X2xpc3QucHVzaCh2ZXJ0ZXgyKTtcclxuICB2ZXJ0ZXhfbGlzdC5wdXNoKGIyVmVjMi5NYWtlKDAsIDApKTtcclxuICB2YXIgZml4X2RlZiA9IG5ldyBiMkZpeHR1cmVEZWYoKTtcclxuICBmaXhfZGVmLnNoYXBlID0gbmV3IGIyUG9seWdvblNoYXBlKCk7XHJcbiAgZml4X2RlZi5kZW5zaXR5ID0gZGVuc2l0eTtcclxuICBmaXhfZGVmLmZyaWN0aW9uID0gMTA7XHJcbiAgZml4X2RlZi5yZXN0aXR1dGlvbiA9IDAuMjtcclxuICBmaXhfZGVmLmZpbHRlci5ncm91cEluZGV4ID0gLTE7XHJcbiAgZml4X2RlZi5zaGFwZS5TZXRBc0FycmF5KHZlcnRleF9saXN0LCAzKTtcclxuXHJcbiAgYm9keS5DcmVhdGVGaXh0dXJlKGZpeF9kZWYpO1xyXG59XHJcblxyXG5mdW5jdGlvbiBjcmVhdGVXaGVlbCh3b3JsZCwgcmFkaXVzLCBkZW5zaXR5KSB7XHJcbiAgdmFyIGJvZHlfZGVmID0gbmV3IGIyQm9keURlZigpO1xyXG4gIGJvZHlfZGVmLnR5cGUgPSBiMkJvZHkuYjJfZHluYW1pY0JvZHk7XHJcbiAgYm9keV9kZWYucG9zaXRpb24uU2V0KDAsIDApO1xyXG5cclxuICB2YXIgYm9keSA9IHdvcmxkLkNyZWF0ZUJvZHkoYm9keV9kZWYpO1xyXG5cclxuICB2YXIgZml4X2RlZiA9IG5ldyBiMkZpeHR1cmVEZWYoKTtcclxuICBmaXhfZGVmLnNoYXBlID0gbmV3IGIyQ2lyY2xlU2hhcGUocmFkaXVzKTtcclxuICBmaXhfZGVmLmRlbnNpdHkgPSBkZW5zaXR5O1xyXG4gIGZpeF9kZWYuZnJpY3Rpb24gPSAxO1xyXG4gIGZpeF9kZWYucmVzdGl0dXRpb24gPSAwLjI7XHJcbiAgZml4X2RlZi5maWx0ZXIuZ3JvdXBJbmRleCA9IC0xO1xyXG5cclxuICBib2R5LkNyZWF0ZUZpeHR1cmUoZml4X2RlZik7XHJcbiAgcmV0dXJuIGJvZHk7XHJcbn1cclxuIiwiXHJcblxyXG5tb2R1bGUuZXhwb3J0cyA9IHtcclxuICBnZXRJbml0aWFsU3RhdGU6IGdldEluaXRpYWxTdGF0ZSxcclxuICB1cGRhdGVTdGF0ZTogdXBkYXRlU3RhdGUsXHJcbiAgZ2V0U3RhdHVzOiBnZXRTdGF0dXMsXHJcbiAgY2FsY3VsYXRlU2NvcmU6IGNhbGN1bGF0ZVNjb3JlLFxyXG59O1xyXG5cclxuZnVuY3Rpb24gZ2V0SW5pdGlhbFN0YXRlKHdvcmxkX2RlZil7XHJcbiAgcmV0dXJuIHtcclxuICAgIGZyYW1lczogMCxcclxuICAgIGhlYWx0aDogd29ybGRfZGVmLm1heF9jYXJfaGVhbHRoLFxyXG4gICAgbWF4UG9zaXRpb255OiAwLFxyXG4gICAgbWluUG9zaXRpb255OiAwLFxyXG4gICAgbWF4UG9zaXRpb254OiAwLFxyXG4gIH07XHJcbn1cclxuXHJcbmZ1bmN0aW9uIHVwZGF0ZVN0YXRlKGNvbnN0YW50cywgd29ybGRDb25zdHJ1Y3QsIHN0YXRlKXtcclxuICBpZihzdGF0ZS5oZWFsdGggPD0gMCl7XHJcbiAgICB0aHJvdyBuZXcgRXJyb3IoXCJBbHJlYWR5IERlYWRcIik7XHJcbiAgfVxyXG4gIGlmKHN0YXRlLm1heFBvc2l0aW9ueCA+IGNvbnN0YW50cy5maW5pc2hMaW5lKXtcclxuICAgIHRocm93IG5ldyBFcnJvcihcImFscmVhZHkgRmluaXNoZWRcIik7XHJcbiAgfVxyXG5cclxuICAvLyBjb25zb2xlLmxvZyhzdGF0ZSk7XHJcbiAgLy8gY2hlY2sgaGVhbHRoXHJcbiAgdmFyIHBvc2l0aW9uID0gd29ybGRDb25zdHJ1Y3QuY2hhc3Npcy5HZXRQb3NpdGlvbigpO1xyXG4gIC8vIGNoZWNrIGlmIGNhciByZWFjaGVkIGVuZCBvZiB0aGUgcGF0aFxyXG4gIHZhciBuZXh0U3RhdGUgPSB7XHJcbiAgICBmcmFtZXM6IHN0YXRlLmZyYW1lcyArIDEsXHJcbiAgICBtYXhQb3NpdGlvbng6IHBvc2l0aW9uLnggPiBzdGF0ZS5tYXhQb3NpdGlvbnggPyBwb3NpdGlvbi54IDogc3RhdGUubWF4UG9zaXRpb254LFxyXG4gICAgbWF4UG9zaXRpb255OiBwb3NpdGlvbi55ID4gc3RhdGUubWF4UG9zaXRpb255ID8gcG9zaXRpb24ueSA6IHN0YXRlLm1heFBvc2l0aW9ueSxcclxuICAgIG1pblBvc2l0aW9ueTogcG9zaXRpb24ueSA8IHN0YXRlLm1pblBvc2l0aW9ueSA/IHBvc2l0aW9uLnkgOiBzdGF0ZS5taW5Qb3NpdGlvbnlcclxuICB9O1xyXG5cclxuICBpZiAocG9zaXRpb24ueCA+IGNvbnN0YW50cy5maW5pc2hMaW5lKSB7XHJcbiAgICByZXR1cm4gbmV4dFN0YXRlO1xyXG4gIH1cclxuXHJcbiAgaWYgKHBvc2l0aW9uLnggPiBzdGF0ZS5tYXhQb3NpdGlvbnggKyAwLjAyKSB7XHJcbiAgICBuZXh0U3RhdGUuaGVhbHRoID0gY29uc3RhbnRzLm1heF9jYXJfaGVhbHRoO1xyXG4gICAgcmV0dXJuIG5leHRTdGF0ZTtcclxuICB9XHJcbiAgbmV4dFN0YXRlLmhlYWx0aCA9IHN0YXRlLmhlYWx0aCAtIDE7XHJcbiAgaWYgKE1hdGguYWJzKHdvcmxkQ29uc3RydWN0LmNoYXNzaXMuR2V0TGluZWFyVmVsb2NpdHkoKS54KSA8IDAuMDAxKSB7XHJcbiAgICBuZXh0U3RhdGUuaGVhbHRoIC09IDU7XHJcbiAgfVxyXG4gIHJldHVybiBuZXh0U3RhdGU7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGdldFN0YXR1cyhzdGF0ZSwgY29uc3RhbnRzKXtcclxuICBpZihoYXNGYWlsZWQoc3RhdGUsIGNvbnN0YW50cykpIHJldHVybiAtMTtcclxuICBpZihoYXNTdWNjZXNzKHN0YXRlLCBjb25zdGFudHMpKSByZXR1cm4gMTtcclxuICByZXR1cm4gMDtcclxufVxyXG5cclxuZnVuY3Rpb24gaGFzRmFpbGVkKHN0YXRlIC8qLCBjb25zdGFudHMgKi8pe1xyXG4gIHJldHVybiBzdGF0ZS5oZWFsdGggPD0gMDtcclxufVxyXG5mdW5jdGlvbiBoYXNTdWNjZXNzKHN0YXRlLCBjb25zdGFudHMpe1xyXG4gIHJldHVybiBzdGF0ZS5tYXhQb3NpdGlvbnggPiBjb25zdGFudHMuZmluaXNoTGluZTtcclxufVxyXG5cclxuZnVuY3Rpb24gY2FsY3VsYXRlU2NvcmUoc3RhdGUsIGNvbnN0YW50cyl7XHJcbiAgdmFyIGF2Z3NwZWVkID0gKHN0YXRlLm1heFBvc2l0aW9ueCAvIHN0YXRlLmZyYW1lcykgKiBjb25zdGFudHMuYm94MmRmcHM7XHJcbiAgdmFyIHBvc2l0aW9uID0gc3RhdGUubWF4UG9zaXRpb254O1xyXG4gIHZhciBzY29yZSA9IHBvc2l0aW9uICsgYXZnc3BlZWQ7XHJcbiAgcmV0dXJuIHtcclxuICAgIHY6IHNjb3JlLFxyXG4gICAgczogYXZnc3BlZWQsXHJcbiAgICB4OiBwb3NpdGlvbixcclxuICAgIHk6IHN0YXRlLm1heFBvc2l0aW9ueSxcclxuICAgIHkyOiBzdGF0ZS5taW5Qb3NpdGlvbnlcclxuICB9XHJcbn1cclxuIiwiLyogZ2xvYmFscyBkb2N1bWVudCAqL1xyXG5cclxudmFyIHJ1biA9IHJlcXVpcmUoXCIuLi9jYXItc2NoZW1hL3J1blwiKTtcclxuXHJcbi8qID09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT0gKi9cclxuLyogPT09IENhciA9PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PSAqL1xyXG52YXIgY3dfQ2FyID0gZnVuY3Rpb24gKCkge1xyXG4gIHRoaXMuX19jb25zdHJ1Y3Rvci5hcHBseSh0aGlzLCBhcmd1bWVudHMpO1xyXG59XHJcblxyXG5jd19DYXIucHJvdG90eXBlLl9fY29uc3RydWN0b3IgPSBmdW5jdGlvbiAoY2FyKSB7XHJcbiAgdGhpcy5jYXIgPSBjYXI7XHJcbiAgdGhpcy5jYXJfZGVmID0gY2FyLmRlZjtcclxuICB2YXIgY2FyX2RlZiA9IHRoaXMuY2FyX2RlZjtcclxuXHJcbiAgdGhpcy5mcmFtZXMgPSAwO1xyXG4gIHRoaXMuYWxpdmUgPSB0cnVlO1xyXG4gIHRoaXMuaXNfZWxpdGUgPSBjYXIuZGVmLmlzX2VsaXRlO1xyXG4gIHRoaXMuaGVhbHRoQmFyID0gZG9jdW1lbnQuZ2V0RWxlbWVudEJ5SWQoXCJoZWFsdGhcIiArIGNhcl9kZWYuaW5kZXgpLnN0eWxlO1xyXG4gIHRoaXMuaGVhbHRoQmFyVGV4dCA9IGRvY3VtZW50LmdldEVsZW1lbnRCeUlkKFwiaGVhbHRoXCIgKyBjYXJfZGVmLmluZGV4KS5uZXh0U2libGluZy5uZXh0U2libGluZztcclxuICB0aGlzLmhlYWx0aEJhclRleHQuaW5uZXJIVE1MID0gY2FyX2RlZi5pbmRleDtcclxuICB0aGlzLm1pbmltYXBtYXJrZXIgPSBkb2N1bWVudC5nZXRFbGVtZW50QnlJZChcImJhclwiICsgY2FyX2RlZi5pbmRleCk7XHJcblxyXG4gIGlmICh0aGlzLmlzX2VsaXRlKSB7XHJcbiAgICB0aGlzLmhlYWx0aEJhci5iYWNrZ3JvdW5kQ29sb3IgPSBcIiMzRjcyQUZcIjtcclxuICAgIHRoaXMubWluaW1hcG1hcmtlci5zdHlsZS5ib3JkZXJMZWZ0ID0gXCIxcHggc29saWQgIzNGNzJBRlwiO1xyXG4gICAgdGhpcy5taW5pbWFwbWFya2VyLmlubmVySFRNTCA9IGNhcl9kZWYuaW5kZXg7XHJcbiAgfSBlbHNlIHtcclxuICAgIHRoaXMuaGVhbHRoQmFyLmJhY2tncm91bmRDb2xvciA9IFwiI0Y3Qzg3M1wiO1xyXG4gICAgdGhpcy5taW5pbWFwbWFya2VyLnN0eWxlLmJvcmRlckxlZnQgPSBcIjFweCBzb2xpZCAjRjdDODczXCI7XHJcbiAgICB0aGlzLm1pbmltYXBtYXJrZXIuaW5uZXJIVE1MID0gY2FyX2RlZi5pbmRleDtcclxuICB9XHJcblxyXG59XHJcblxyXG5jd19DYXIucHJvdG90eXBlLmdldFBvc2l0aW9uID0gZnVuY3Rpb24gKCkge1xyXG4gIHJldHVybiB0aGlzLmNhci5jYXIuY2hhc3Npcy5HZXRQb3NpdGlvbigpO1xyXG59XHJcblxyXG5jd19DYXIucHJvdG90eXBlLmtpbGwgPSBmdW5jdGlvbiAoY3VycmVudFJ1bm5lciwgY29uc3RhbnRzKSB7XHJcbiAgdGhpcy5taW5pbWFwbWFya2VyLnN0eWxlLmJvcmRlckxlZnQgPSBcIjFweCBzb2xpZCAjM0Y3MkFGXCI7XHJcbiAgdmFyIGZpbmlzaExpbmUgPSBjdXJyZW50UnVubmVyLnNjZW5lLmZpbmlzaExpbmVcclxuICB2YXIgbWF4X2Nhcl9oZWFsdGggPSBjb25zdGFudHMubWF4X2Nhcl9oZWFsdGg7XHJcbiAgdmFyIHN0YXR1cyA9IHJ1bi5nZXRTdGF0dXModGhpcy5jYXIuc3RhdGUsIHtcclxuICAgIGZpbmlzaExpbmU6IGZpbmlzaExpbmUsXHJcbiAgICBtYXhfY2FyX2hlYWx0aDogbWF4X2Nhcl9oZWFsdGgsXHJcbiAgfSlcclxuICBzd2l0Y2goc3RhdHVzKXtcclxuICAgIGNhc2UgMToge1xyXG4gICAgICB0aGlzLmhlYWx0aEJhci53aWR0aCA9IFwiMFwiO1xyXG4gICAgICBicmVha1xyXG4gICAgfVxyXG4gICAgY2FzZSAtMToge1xyXG4gICAgICB0aGlzLmhlYWx0aEJhclRleHQuaW5uZXJIVE1MID0gXCImZGFnZ2VyO1wiO1xyXG4gICAgICB0aGlzLmhlYWx0aEJhci53aWR0aCA9IFwiMFwiO1xyXG4gICAgICBicmVha1xyXG4gICAgfVxyXG4gIH1cclxuICB0aGlzLmFsaXZlID0gZmFsc2U7XHJcblxyXG59XHJcblxyXG5tb2R1bGUuZXhwb3J0cyA9IGN3X0NhcjtcclxuIiwiXHJcbnZhciBjd19kcmF3VmlydHVhbFBvbHkgPSByZXF1aXJlKFwiLi9kcmF3LXZpcnR1YWwtcG9seVwiKTtcclxudmFyIGN3X2RyYXdDaXJjbGUgPSByZXF1aXJlKFwiLi9kcmF3LWNpcmNsZVwiKTtcclxuXHJcbm1vZHVsZS5leHBvcnRzID0gZnVuY3Rpb24oY2FyX2NvbnN0YW50cywgbXlDYXIsIGNhbWVyYSwgY3R4KXtcclxuICB2YXIgY2FtZXJhX3ggPSBjYW1lcmEucG9zLng7XHJcbiAgdmFyIHpvb20gPSBjYW1lcmEuem9vbTtcclxuXHJcbiAgdmFyIHdoZWVsTWluRGVuc2l0eSA9IGNhcl9jb25zdGFudHMud2hlZWxNaW5EZW5zaXR5XHJcbiAgdmFyIHdoZWVsRGVuc2l0eVJhbmdlID0gY2FyX2NvbnN0YW50cy53aGVlbERlbnNpdHlSYW5nZVxyXG5cclxuICBpZiAoIW15Q2FyLmFsaXZlKSB7XHJcbiAgICByZXR1cm47XHJcbiAgfVxyXG4gIHZhciBteUNhclBvcyA9IG15Q2FyLmdldFBvc2l0aW9uKCk7XHJcblxyXG4gIGlmIChteUNhclBvcy54IDwgKGNhbWVyYV94IC0gNSkpIHtcclxuICAgIC8vIHRvbyBmYXIgYmVoaW5kLCBkb24ndCBkcmF3XHJcbiAgICByZXR1cm47XHJcbiAgfVxyXG5cclxuICBjdHguc3Ryb2tlU3R5bGUgPSBcIiM0NDRcIjtcclxuICBjdHgubGluZVdpZHRoID0gMSAvIHpvb207XHJcblxyXG4gIHZhciB3aGVlbHMgPSBteUNhci5jYXIuY2FyLndoZWVscztcclxuXHJcbiAgZm9yICh2YXIgaSA9IDA7IGkgPCB3aGVlbHMubGVuZ3RoOyBpKyspIHtcclxuICAgIHZhciBiID0gd2hlZWxzW2ldO1xyXG4gICAgZm9yICh2YXIgZiA9IGIuR2V0Rml4dHVyZUxpc3QoKTsgZjsgZiA9IGYubV9uZXh0KSB7XHJcbiAgICAgIHZhciBzID0gZi5HZXRTaGFwZSgpO1xyXG4gICAgICB2YXIgY29sb3IgPSBNYXRoLnJvdW5kKDI1NSAtICgyNTUgKiAoZi5tX2RlbnNpdHkgLSB3aGVlbE1pbkRlbnNpdHkpKSAvIHdoZWVsRGVuc2l0eVJhbmdlKS50b1N0cmluZygpO1xyXG4gICAgICB2YXIgcmdiY29sb3IgPSBcInJnYihcIiArIGNvbG9yICsgXCIsXCIgKyBjb2xvciArIFwiLFwiICsgY29sb3IgKyBcIilcIjtcclxuICAgICAgY3dfZHJhd0NpcmNsZShjdHgsIGIsIHMubV9wLCBzLm1fcmFkaXVzLCBiLm1fc3dlZXAuYSwgcmdiY29sb3IpO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgaWYgKG15Q2FyLmlzX2VsaXRlKSB7XHJcbiAgICBjdHguc3Ryb2tlU3R5bGUgPSBcIiMzRjcyQUZcIjtcclxuICAgIGN0eC5maWxsU3R5bGUgPSBcIiNEQkUyRUZcIjtcclxuICB9IGVsc2Uge1xyXG4gICAgY3R4LnN0cm9rZVN0eWxlID0gXCIjRjdDODczXCI7XHJcbiAgICBjdHguZmlsbFN0eWxlID0gXCIjRkFFQkNEXCI7XHJcbiAgfVxyXG4gIGN0eC5iZWdpblBhdGgoKTtcclxuXHJcbiAgdmFyIGNoYXNzaXMgPSBteUNhci5jYXIuY2FyLmNoYXNzaXM7XHJcblxyXG4gIGZvciAoZiA9IGNoYXNzaXMuR2V0Rml4dHVyZUxpc3QoKTsgZjsgZiA9IGYubV9uZXh0KSB7XHJcbiAgICB2YXIgY3MgPSBmLkdldFNoYXBlKCk7XHJcbiAgICBjd19kcmF3VmlydHVhbFBvbHkoY3R4LCBjaGFzc2lzLCBjcy5tX3ZlcnRpY2VzLCBjcy5tX3ZlcnRleENvdW50KTtcclxuICB9XHJcbiAgY3R4LmZpbGwoKTtcclxuICBjdHguc3Ryb2tlKCk7XHJcbn1cclxuIiwiXHJcbm1vZHVsZS5leHBvcnRzID0gY3dfZHJhd0NpcmNsZTtcclxuXHJcbmZ1bmN0aW9uIGN3X2RyYXdDaXJjbGUoY3R4LCBib2R5LCBjZW50ZXIsIHJhZGl1cywgYW5nbGUsIGNvbG9yKSB7XHJcbiAgdmFyIHAgPSBib2R5LkdldFdvcmxkUG9pbnQoY2VudGVyKTtcclxuICBjdHguZmlsbFN0eWxlID0gY29sb3I7XHJcblxyXG4gIGN0eC5iZWdpblBhdGgoKTtcclxuICBjdHguYXJjKHAueCwgcC55LCByYWRpdXMsIDAsIDIgKiBNYXRoLlBJLCB0cnVlKTtcclxuXHJcbiAgY3R4Lm1vdmVUbyhwLngsIHAueSk7XHJcbiAgY3R4LmxpbmVUbyhwLnggKyByYWRpdXMgKiBNYXRoLmNvcyhhbmdsZSksIHAueSArIHJhZGl1cyAqIE1hdGguc2luKGFuZ2xlKSk7XHJcblxyXG4gIGN0eC5maWxsKCk7XHJcbiAgY3R4LnN0cm9rZSgpO1xyXG59XHJcbiIsInZhciBjd19kcmF3VmlydHVhbFBvbHkgPSByZXF1aXJlKFwiLi9kcmF3LXZpcnR1YWwtcG9seVwiKTtcclxubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbihjdHgsIGNhbWVyYSwgY3dfZmxvb3JUaWxlcykge1xyXG4gIHZhciBjYW1lcmFfeCA9IGNhbWVyYS5wb3MueDtcclxuICB2YXIgem9vbSA9IGNhbWVyYS56b29tO1xyXG4gIGN0eC5zdHJva2VTdHlsZSA9IFwiIzAwMFwiO1xyXG4gIGN0eC5maWxsU3R5bGUgPSBcIiM3NzdcIjtcclxuICBjdHgubGluZVdpZHRoID0gMSAvIHpvb207XHJcbiAgY3R4LmJlZ2luUGF0aCgpO1xyXG5cclxuICB2YXIgaztcclxuICBpZihjYW1lcmEucG9zLnggLSAxMCA+IDApe1xyXG4gICAgayA9IE1hdGguZmxvb3IoKGNhbWVyYS5wb3MueCAtIDEwKSAvIDEuNSk7XHJcbiAgfSBlbHNlIHtcclxuICAgIGsgPSAwO1xyXG4gIH1cclxuXHJcbiAgLy8gY29uc29sZS5sb2coayk7XHJcblxyXG4gIG91dGVyX2xvb3A6XHJcbiAgICBmb3IgKGs7IGsgPCBjd19mbG9vclRpbGVzLmxlbmd0aDsgaysrKSB7XHJcbiAgICAgIHZhciBiID0gY3dfZmxvb3JUaWxlc1trXTtcclxuICAgICAgZm9yICh2YXIgZiA9IGIuR2V0Rml4dHVyZUxpc3QoKTsgZjsgZiA9IGYubV9uZXh0KSB7XHJcbiAgICAgICAgdmFyIHMgPSBmLkdldFNoYXBlKCk7XHJcbiAgICAgICAgdmFyIHNoYXBlUG9zaXRpb24gPSBiLkdldFdvcmxkUG9pbnQocy5tX3ZlcnRpY2VzWzBdKS54O1xyXG4gICAgICAgIGlmICgoc2hhcGVQb3NpdGlvbiA+IChjYW1lcmFfeCAtIDUpKSAmJiAoc2hhcGVQb3NpdGlvbiA8IChjYW1lcmFfeCArIDEwKSkpIHtcclxuICAgICAgICAgIGN3X2RyYXdWaXJ0dWFsUG9seShjdHgsIGIsIHMubV92ZXJ0aWNlcywgcy5tX3ZlcnRleENvdW50KTtcclxuICAgICAgICB9XHJcbiAgICAgICAgaWYgKHNoYXBlUG9zaXRpb24gPiBjYW1lcmFfeCArIDEwKSB7XHJcbiAgICAgICAgICBicmVhayBvdXRlcl9sb29wO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG4gIGN0eC5maWxsKCk7XHJcbiAgY3R4LnN0cm9rZSgpO1xyXG59XHJcbiIsIlxyXG5cclxubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbihjdHgsIGJvZHksIHZ0eCwgbl92dHgpIHtcclxuICAvLyBzZXQgc3Ryb2tlc3R5bGUgYW5kIGZpbGxzdHlsZSBiZWZvcmUgY2FsbFxyXG4gIC8vIGNhbGwgYmVnaW5QYXRoIGJlZm9yZSBjYWxsXHJcblxyXG4gIHZhciBwMCA9IGJvZHkuR2V0V29ybGRQb2ludCh2dHhbMF0pO1xyXG4gIGN0eC5tb3ZlVG8ocDAueCwgcDAueSk7XHJcbiAgZm9yICh2YXIgaSA9IDE7IGkgPCBuX3Z0eDsgaSsrKSB7XHJcbiAgICB2YXIgcCA9IGJvZHkuR2V0V29ybGRQb2ludCh2dHhbaV0pO1xyXG4gICAgY3R4LmxpbmVUbyhwLngsIHAueSk7XHJcbiAgfVxyXG4gIGN0eC5saW5lVG8ocDAueCwgcDAueSk7XHJcbn1cclxuIiwidmFyIHNjYXR0ZXJQbG90ID0gcmVxdWlyZShcIi4vc2NhdHRlci1wbG90XCIpO1xyXG5cclxubW9kdWxlLmV4cG9ydHMgPSB7XHJcbiAgcGxvdEdyYXBoczogZnVuY3Rpb24oZ3JhcGhFbGVtLCB0b3BTY29yZXNFbGVtLCBzY2F0dGVyUGxvdEVsZW0sIGxhc3RTdGF0ZSwgc2NvcmVzLCBjb25maWcpIHtcclxuICAgIGxhc3RTdGF0ZSA9IGxhc3RTdGF0ZSB8fCB7fTtcclxuICAgIHZhciBnZW5lcmF0aW9uU2l6ZSA9IHNjb3Jlcy5sZW5ndGhcclxuICAgIHZhciBncmFwaGNhbnZhcyA9IGdyYXBoRWxlbTtcclxuICAgIHZhciBncmFwaGN0eCA9IGdyYXBoY2FudmFzLmdldENvbnRleHQoXCIyZFwiKTtcclxuICAgIHZhciBncmFwaHdpZHRoID0gNDAwO1xyXG4gICAgdmFyIGdyYXBoaGVpZ2h0ID0gMjUwO1xyXG4gICAgdmFyIG5leHRTdGF0ZSA9IGN3X3N0b3JlR3JhcGhTY29yZXMoXHJcbiAgICAgIGxhc3RTdGF0ZSwgc2NvcmVzLCBnZW5lcmF0aW9uU2l6ZVxyXG4gICAgKTtcclxuICAgIGNvbnNvbGUubG9nKHNjb3JlcywgbmV4dFN0YXRlKTtcclxuICAgIGN3X2NsZWFyR3JhcGhpY3MoZ3JhcGhjYW52YXMsIGdyYXBoY3R4LCBncmFwaHdpZHRoLCBncmFwaGhlaWdodCk7XHJcbiAgICBjd19wbG90QXZlcmFnZShuZXh0U3RhdGUsIGdyYXBoY3R4KTtcclxuICAgIGN3X3Bsb3RFbGl0ZShuZXh0U3RhdGUsIGdyYXBoY3R4KTtcclxuICAgIGN3X3Bsb3RUb3AobmV4dFN0YXRlLCBncmFwaGN0eCk7XHJcbiAgICBjd19saXN0VG9wU2NvcmVzKHRvcFNjb3Jlc0VsZW0sIG5leHRTdGF0ZSk7XHJcbiAgICBuZXh0U3RhdGUuc2NhdHRlckdyYXBoID0gZHJhd0FsbFJlc3VsdHMoXHJcbiAgICAgIHNjYXR0ZXJQbG90RWxlbSwgY29uZmlnLCBuZXh0U3RhdGUsIGxhc3RTdGF0ZS5zY2F0dGVyR3JhcGhcclxuICAgICk7XHJcbiAgICByZXR1cm4gbmV4dFN0YXRlO1xyXG4gIH0sXHJcbiAgY2xlYXJHcmFwaGljczogZnVuY3Rpb24oZ3JhcGhFbGVtKSB7XHJcbiAgICB2YXIgZ3JhcGhjYW52YXMgPSBncmFwaEVsZW07XHJcbiAgICB2YXIgZ3JhcGhjdHggPSBncmFwaGNhbnZhcy5nZXRDb250ZXh0KFwiMmRcIik7XHJcbiAgICB2YXIgZ3JhcGh3aWR0aCA9IDQwMDtcclxuICAgIHZhciBncmFwaGhlaWdodCA9IDI1MDtcclxuICAgIGN3X2NsZWFyR3JhcGhpY3MoZ3JhcGhjYW52YXMsIGdyYXBoY3R4LCBncmFwaHdpZHRoLCBncmFwaGhlaWdodCk7XHJcbiAgfVxyXG59O1xyXG5cclxuXHJcbmZ1bmN0aW9uIGN3X3N0b3JlR3JhcGhTY29yZXMobGFzdFN0YXRlLCBjd19jYXJTY29yZXMsIGdlbmVyYXRpb25TaXplKSB7XHJcbiAgY29uc29sZS5sb2coY3dfY2FyU2NvcmVzKTtcclxuICByZXR1cm4ge1xyXG4gICAgY3dfdG9wU2NvcmVzOiAobGFzdFN0YXRlLmN3X3RvcFNjb3JlcyB8fCBbXSlcclxuICAgIC5jb25jYXQoW2N3X2NhclNjb3Jlc1swXS5zY29yZV0pLFxyXG4gICAgY3dfZ3JhcGhBdmVyYWdlOiAobGFzdFN0YXRlLmN3X2dyYXBoQXZlcmFnZSB8fCBbXSkuY29uY2F0KFtcclxuICAgICAgY3dfYXZlcmFnZShjd19jYXJTY29yZXMsIGdlbmVyYXRpb25TaXplKVxyXG4gICAgXSksXHJcbiAgICBjd19ncmFwaEVsaXRlOiAobGFzdFN0YXRlLmN3X2dyYXBoRWxpdGUgfHwgW10pLmNvbmNhdChbXHJcbiAgICAgIGN3X2VsaXRlYXZlcmFnZShjd19jYXJTY29yZXMsIGdlbmVyYXRpb25TaXplKVxyXG4gICAgXSksXHJcbiAgICBjd19ncmFwaFRvcDogKGxhc3RTdGF0ZS5jd19ncmFwaFRvcCB8fCBbXSkuY29uY2F0KFtcclxuICAgICAgY3dfY2FyU2NvcmVzWzBdLnNjb3JlLnZcclxuICAgIF0pLFxyXG4gICAgYWxsUmVzdWx0czogKGxhc3RTdGF0ZS5hbGxSZXN1bHRzIHx8IFtdKS5jb25jYXQoY3dfY2FyU2NvcmVzKSxcclxuICB9XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGN3X3Bsb3RUb3Aoc3RhdGUsIGdyYXBoY3R4KSB7XHJcbiAgdmFyIGN3X2dyYXBoVG9wID0gc3RhdGUuY3dfZ3JhcGhUb3A7XHJcbiAgdmFyIGdyYXBoc2l6ZSA9IGN3X2dyYXBoVG9wLmxlbmd0aDtcclxuICBncmFwaGN0eC5zdHJva2VTdHlsZSA9IFwiI0M4M0IzQlwiO1xyXG4gIGdyYXBoY3R4LmJlZ2luUGF0aCgpO1xyXG4gIGdyYXBoY3R4Lm1vdmVUbygwLCAwKTtcclxuICBmb3IgKHZhciBrID0gMDsgayA8IGdyYXBoc2l6ZTsgaysrKSB7XHJcbiAgICBncmFwaGN0eC5saW5lVG8oNDAwICogKGsgKyAxKSAvIGdyYXBoc2l6ZSwgY3dfZ3JhcGhUb3Bba10pO1xyXG4gIH1cclxuICBncmFwaGN0eC5zdHJva2UoKTtcclxufVxyXG5cclxuZnVuY3Rpb24gY3dfcGxvdEVsaXRlKHN0YXRlLCBncmFwaGN0eCkge1xyXG4gIHZhciBjd19ncmFwaEVsaXRlID0gc3RhdGUuY3dfZ3JhcGhFbGl0ZTtcclxuICB2YXIgZ3JhcGhzaXplID0gY3dfZ3JhcGhFbGl0ZS5sZW5ndGg7XHJcbiAgZ3JhcGhjdHguc3Ryb2tlU3R5bGUgPSBcIiM3QkM3NERcIjtcclxuICBncmFwaGN0eC5iZWdpblBhdGgoKTtcclxuICBncmFwaGN0eC5tb3ZlVG8oMCwgMCk7XHJcbiAgZm9yICh2YXIgayA9IDA7IGsgPCBncmFwaHNpemU7IGsrKykge1xyXG4gICAgZ3JhcGhjdHgubGluZVRvKDQwMCAqIChrICsgMSkgLyBncmFwaHNpemUsIGN3X2dyYXBoRWxpdGVba10pO1xyXG4gIH1cclxuICBncmFwaGN0eC5zdHJva2UoKTtcclxufVxyXG5cclxuZnVuY3Rpb24gY3dfcGxvdEF2ZXJhZ2Uoc3RhdGUsIGdyYXBoY3R4KSB7XHJcbiAgdmFyIGN3X2dyYXBoQXZlcmFnZSA9IHN0YXRlLmN3X2dyYXBoQXZlcmFnZTtcclxuICB2YXIgZ3JhcGhzaXplID0gY3dfZ3JhcGhBdmVyYWdlLmxlbmd0aDtcclxuICBncmFwaGN0eC5zdHJva2VTdHlsZSA9IFwiIzNGNzJBRlwiO1xyXG4gIGdyYXBoY3R4LmJlZ2luUGF0aCgpO1xyXG4gIGdyYXBoY3R4Lm1vdmVUbygwLCAwKTtcclxuICBmb3IgKHZhciBrID0gMDsgayA8IGdyYXBoc2l6ZTsgaysrKSB7XHJcbiAgICBncmFwaGN0eC5saW5lVG8oNDAwICogKGsgKyAxKSAvIGdyYXBoc2l6ZSwgY3dfZ3JhcGhBdmVyYWdlW2tdKTtcclxuICB9XHJcbiAgZ3JhcGhjdHguc3Ryb2tlKCk7XHJcbn1cclxuXHJcblxyXG5mdW5jdGlvbiBjd19lbGl0ZWF2ZXJhZ2Uoc2NvcmVzLCBnZW5lcmF0aW9uU2l6ZSkge1xyXG4gIHZhciBzdW0gPSAwO1xyXG4gIGZvciAodmFyIGsgPSAwOyBrIDwgTWF0aC5mbG9vcihnZW5lcmF0aW9uU2l6ZSAvIDIpOyBrKyspIHtcclxuICAgIHN1bSArPSBzY29yZXNba10uc2NvcmUudjtcclxuICB9XHJcbiAgcmV0dXJuIHN1bSAvIE1hdGguZmxvb3IoZ2VuZXJhdGlvblNpemUgLyAyKTtcclxufVxyXG5cclxuZnVuY3Rpb24gY3dfYXZlcmFnZShzY29yZXMsIGdlbmVyYXRpb25TaXplKSB7XHJcbiAgdmFyIHN1bSA9IDA7XHJcbiAgZm9yICh2YXIgayA9IDA7IGsgPCBnZW5lcmF0aW9uU2l6ZTsgaysrKSB7XHJcbiAgICBzdW0gKz0gc2NvcmVzW2tdLnNjb3JlLnY7XHJcbiAgfVxyXG4gIHJldHVybiBzdW0gLyBnZW5lcmF0aW9uU2l6ZTtcclxufVxyXG5cclxuZnVuY3Rpb24gY3dfY2xlYXJHcmFwaGljcyhncmFwaGNhbnZhcywgZ3JhcGhjdHgsIGdyYXBod2lkdGgsIGdyYXBoaGVpZ2h0KSB7XHJcbiAgZ3JhcGhjYW52YXMud2lkdGggPSBncmFwaGNhbnZhcy53aWR0aDtcclxuICBncmFwaGN0eC50cmFuc2xhdGUoMCwgZ3JhcGhoZWlnaHQpO1xyXG4gIGdyYXBoY3R4LnNjYWxlKDEsIC0xKTtcclxuICBncmFwaGN0eC5saW5lV2lkdGggPSAxO1xyXG4gIGdyYXBoY3R4LnN0cm9rZVN0eWxlID0gXCIjM0Y3MkFGXCI7XHJcbiAgZ3JhcGhjdHguYmVnaW5QYXRoKCk7XHJcbiAgZ3JhcGhjdHgubW92ZVRvKDAsIGdyYXBoaGVpZ2h0IC8gMik7XHJcbiAgZ3JhcGhjdHgubGluZVRvKGdyYXBod2lkdGgsIGdyYXBoaGVpZ2h0IC8gMik7XHJcbiAgZ3JhcGhjdHgubW92ZVRvKDAsIGdyYXBoaGVpZ2h0IC8gNCk7XHJcbiAgZ3JhcGhjdHgubGluZVRvKGdyYXBod2lkdGgsIGdyYXBoaGVpZ2h0IC8gNCk7XHJcbiAgZ3JhcGhjdHgubW92ZVRvKDAsIGdyYXBoaGVpZ2h0ICogMyAvIDQpO1xyXG4gIGdyYXBoY3R4LmxpbmVUbyhncmFwaHdpZHRoLCBncmFwaGhlaWdodCAqIDMgLyA0KTtcclxuICBncmFwaGN0eC5zdHJva2UoKTtcclxufVxyXG5cclxuZnVuY3Rpb24gY3dfbGlzdFRvcFNjb3JlcyhlbGVtLCBzdGF0ZSkge1xyXG4gIHZhciBjd190b3BTY29yZXMgPSBzdGF0ZS5jd190b3BTY29yZXM7XHJcbiAgdmFyIHRzID0gZWxlbTtcclxuICB0cy5pbm5lckhUTUwgPSBcIjxiPlRvcCBTY29yZXM6PC9iPjxiciAvPlwiO1xyXG4gIGN3X3RvcFNjb3Jlcy5zb3J0KGZ1bmN0aW9uIChhLCBiKSB7XHJcbiAgICBpZiAoYS52ID4gYi52KSB7XHJcbiAgICAgIHJldHVybiAtMVxyXG4gICAgfSBlbHNlIHtcclxuICAgICAgcmV0dXJuIDFcclxuICAgIH1cclxuICB9KTtcclxuXHJcbiAgZm9yICh2YXIgayA9IDA7IGsgPCBNYXRoLm1pbigxMCwgY3dfdG9wU2NvcmVzLmxlbmd0aCk7IGsrKykge1xyXG4gICAgdmFyIHRvcFNjb3JlID0gY3dfdG9wU2NvcmVzW2tdO1xyXG4gICAgLy8gY29uc29sZS5sb2codG9wU2NvcmUpO1xyXG4gICAgdmFyIG4gPSBcIiNcIiArIChrICsgMSkgKyBcIjpcIjtcclxuICAgIHZhciBzY29yZSA9IE1hdGgucm91bmQodG9wU2NvcmUudiAqIDEwMCkgLyAxMDA7XHJcbiAgICB2YXIgZGlzdGFuY2UgPSBcImQ6XCIgKyBNYXRoLnJvdW5kKHRvcFNjb3JlLnggKiAxMDApIC8gMTAwO1xyXG4gICAgdmFyIHlyYW5nZSA9ICBcImg6XCIgKyBNYXRoLnJvdW5kKHRvcFNjb3JlLnkyICogMTAwKSAvIDEwMCArIFwiL1wiICsgTWF0aC5yb3VuZCh0b3BTY29yZS55ICogMTAwKSAvIDEwMCArIFwibVwiO1xyXG4gICAgdmFyIGdlbiA9IFwiKEdlbiBcIiArIGN3X3RvcFNjb3Jlc1trXS5pICsgXCIpXCJcclxuXHJcbiAgICB0cy5pbm5lckhUTUwgKz0gIFtuLCBzY29yZSwgZGlzdGFuY2UsIHlyYW5nZSwgZ2VuXS5qb2luKFwiIFwiKSArIFwiPGJyIC8+XCI7XHJcbiAgfVxyXG59XHJcblxyXG5mdW5jdGlvbiBkcmF3QWxsUmVzdWx0cyhzY2F0dGVyUGxvdEVsZW0sIGNvbmZpZywgYWxsUmVzdWx0cywgcHJldmlvdXNHcmFwaCl7XHJcbiAgaWYoIXNjYXR0ZXJQbG90RWxlbSkgcmV0dXJuO1xyXG4gIHJldHVybiBzY2F0dGVyUGxvdChzY2F0dGVyUGxvdEVsZW0sIGFsbFJlc3VsdHMsIGNvbmZpZy5wcm9wZXJ0eU1hcCwgcHJldmlvdXNHcmFwaClcclxufVxyXG4iLCIvKiBnbG9iYWxzIHZpcyBIaWdoY2hhcnRzICovXHJcblxyXG4vLyBDYWxsZWQgd2hlbiB0aGUgVmlzdWFsaXphdGlvbiBBUEkgaXMgbG9hZGVkLlxyXG5cclxubW9kdWxlLmV4cG9ydHMgPSBoaWdoQ2hhcnRzO1xyXG5mdW5jdGlvbiBoaWdoQ2hhcnRzKGVsZW0sIHNjb3Jlcyl7XHJcbiAgdmFyIGtleXMgPSBPYmplY3Qua2V5cyhzY29yZXNbMF0uZGVmKTtcclxuICBrZXlzID0ga2V5cy5yZWR1Y2UoZnVuY3Rpb24oY3VyQXJyYXksIGtleSl7XHJcbiAgICB2YXIgbCA9IHNjb3Jlc1swXS5kZWZba2V5XS5sZW5ndGg7XHJcbiAgICB2YXIgc3ViQXJyYXkgPSBbXTtcclxuICAgIGZvcih2YXIgaSA9IDA7IGkgPCBsOyBpKyspe1xyXG4gICAgICBzdWJBcnJheS5wdXNoKGtleSArIFwiLlwiICsgaSk7XHJcbiAgICB9XHJcbiAgICByZXR1cm4gY3VyQXJyYXkuY29uY2F0KHN1YkFycmF5KTtcclxuICB9LCBbXSk7XHJcbiAgZnVuY3Rpb24gcmV0cmlldmVWYWx1ZShvYmosIHBhdGgpe1xyXG4gICAgcmV0dXJuIHBhdGguc3BsaXQoXCIuXCIpLnJlZHVjZShmdW5jdGlvbihjdXJWYWx1ZSwga2V5KXtcclxuICAgICAgcmV0dXJuIGN1clZhbHVlW2tleV07XHJcbiAgICB9LCBvYmopO1xyXG4gIH1cclxuXHJcbiAgdmFyIGRhdGFPYmogPSBPYmplY3Qua2V5cyhzY29yZXMpLnJlZHVjZShmdW5jdGlvbihrdiwgc2NvcmUpe1xyXG4gICAga2V5cy5mb3JFYWNoKGZ1bmN0aW9uKGtleSl7XHJcbiAgICAgIGt2W2tleV0uZGF0YS5wdXNoKFtcclxuICAgICAgICByZXRyaWV2ZVZhbHVlKHNjb3JlLmRlZiwga2V5KSwgc2NvcmUuc2NvcmUudlxyXG4gICAgICBdKVxyXG4gICAgfSlcclxuICAgIHJldHVybiBrdjtcclxuICB9LCBrZXlzLnJlZHVjZShmdW5jdGlvbihrdiwga2V5KXtcclxuICAgIGt2W2tleV0gPSB7XHJcbiAgICAgIG5hbWU6IGtleSxcclxuICAgICAgZGF0YTogW10sXHJcbiAgICB9XHJcbiAgICByZXR1cm4ga3Y7XHJcbiAgfSwge30pKVxyXG4gIEhpZ2hjaGFydHMuY2hhcnQoZWxlbS5pZCwge1xyXG4gICAgICBjaGFydDoge1xyXG4gICAgICAgICAgdHlwZTogJ3NjYXR0ZXInLFxyXG4gICAgICAgICAgem9vbVR5cGU6ICd4eSdcclxuICAgICAgfSxcclxuICAgICAgdGl0bGU6IHtcclxuICAgICAgICAgIHRleHQ6ICdQcm9wZXJ0eSBWYWx1ZSB0byBTY29yZSdcclxuICAgICAgfSxcclxuICAgICAgeEF4aXM6IHtcclxuICAgICAgICAgIHRpdGxlOiB7XHJcbiAgICAgICAgICAgICAgZW5hYmxlZDogdHJ1ZSxcclxuICAgICAgICAgICAgICB0ZXh0OiAnTm9ybWFsaXplZCdcclxuICAgICAgICAgIH0sXHJcbiAgICAgICAgICBzdGFydE9uVGljazogdHJ1ZSxcclxuICAgICAgICAgIGVuZE9uVGljazogdHJ1ZSxcclxuICAgICAgICAgIHNob3dMYXN0TGFiZWw6IHRydWVcclxuICAgICAgfSxcclxuICAgICAgeUF4aXM6IHtcclxuICAgICAgICAgIHRpdGxlOiB7XHJcbiAgICAgICAgICAgICAgdGV4dDogJ1Njb3JlJ1xyXG4gICAgICAgICAgfVxyXG4gICAgICB9LFxyXG4gICAgICBsZWdlbmQ6IHtcclxuICAgICAgICAgIGxheW91dDogJ3ZlcnRpY2FsJyxcclxuICAgICAgICAgIGFsaWduOiAnbGVmdCcsXHJcbiAgICAgICAgICB2ZXJ0aWNhbEFsaWduOiAndG9wJyxcclxuICAgICAgICAgIHg6IDEwMCxcclxuICAgICAgICAgIHk6IDcwLFxyXG4gICAgICAgICAgZmxvYXRpbmc6IHRydWUsXHJcbiAgICAgICAgICBiYWNrZ3JvdW5kQ29sb3I6IChIaWdoY2hhcnRzLnRoZW1lICYmIEhpZ2hjaGFydHMudGhlbWUubGVnZW5kQmFja2dyb3VuZENvbG9yKSB8fCAnI0ZGRkZGRicsXHJcbiAgICAgICAgICBib3JkZXJXaWR0aDogMVxyXG4gICAgICB9LFxyXG4gICAgICBwbG90T3B0aW9uczoge1xyXG4gICAgICAgICAgc2NhdHRlcjoge1xyXG4gICAgICAgICAgICAgIG1hcmtlcjoge1xyXG4gICAgICAgICAgICAgICAgICByYWRpdXM6IDUsXHJcbiAgICAgICAgICAgICAgICAgIHN0YXRlczoge1xyXG4gICAgICAgICAgICAgICAgICAgICAgaG92ZXI6IHtcclxuICAgICAgICAgICAgICAgICAgICAgICAgICBlbmFibGVkOiB0cnVlLFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgIGxpbmVDb2xvcjogJ3JnYigxMDAsMTAwLDEwMCknXHJcbiAgICAgICAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICB9LFxyXG4gICAgICAgICAgICAgIHN0YXRlczoge1xyXG4gICAgICAgICAgICAgICAgICBob3Zlcjoge1xyXG4gICAgICAgICAgICAgICAgICAgICAgbWFya2VyOiB7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgZW5hYmxlZDogZmFsc2VcclxuICAgICAgICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgIH0sXHJcbiAgICAgICAgICAgICAgdG9vbHRpcDoge1xyXG4gICAgICAgICAgICAgICAgICBoZWFkZXJGb3JtYXQ6ICc8Yj57c2VyaWVzLm5hbWV9PC9iPjxicj4nLFxyXG4gICAgICAgICAgICAgICAgICBwb2ludEZvcm1hdDogJ3twb2ludC54fSwge3BvaW50Lnl9J1xyXG4gICAgICAgICAgICAgIH1cclxuICAgICAgICAgIH1cclxuICAgICAgfSxcclxuICAgICAgc2VyaWVzOiBrZXlzLm1hcChmdW5jdGlvbihrZXkpe1xyXG4gICAgICAgIHJldHVybiBkYXRhT2JqW2tleV07XHJcbiAgICAgIH0pXHJcbiAgfSk7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIHZpc0NoYXJ0KGVsZW0sIHNjb3JlcywgcHJvcGVydHlNYXAsIGdyYXBoKSB7XHJcblxyXG4gIC8vIENyZWF0ZSBhbmQgcG9wdWxhdGUgYSBkYXRhIHRhYmxlLlxyXG4gIHZhciBkYXRhID0gbmV3IHZpcy5EYXRhU2V0KCk7XHJcbiAgc2NvcmVzLmZvckVhY2goZnVuY3Rpb24oc2NvcmVJbmZvKXtcclxuICAgIGRhdGEuYWRkKHtcclxuICAgICAgeDogZ2V0UHJvcGVydHkoc2NvcmVJbmZvLCBwcm9wZXJ0eU1hcC54KSxcclxuICAgICAgeTogZ2V0UHJvcGVydHkoc2NvcmVJbmZvLCBwcm9wZXJ0eU1hcC54KSxcclxuICAgICAgejogZ2V0UHJvcGVydHkoc2NvcmVJbmZvLCBwcm9wZXJ0eU1hcC56KSxcclxuICAgICAgc3R5bGU6IGdldFByb3BlcnR5KHNjb3JlSW5mbywgcHJvcGVydHlNYXAueiksXHJcbiAgICAgIC8vIGV4dHJhOiBkZWYuYW5jZXN0cnlcclxuICAgIH0pO1xyXG4gIH0pO1xyXG5cclxuICBmdW5jdGlvbiBnZXRQcm9wZXJ0eShpbmZvLCBrZXkpe1xyXG4gICAgaWYoa2V5ID09PSBcInNjb3JlXCIpe1xyXG4gICAgICByZXR1cm4gaW5mby5zY29yZS52XHJcbiAgICB9IGVsc2Uge1xyXG4gICAgICByZXR1cm4gaW5mby5kZWZba2V5XTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIC8vIHNwZWNpZnkgb3B0aW9uc1xyXG4gIHZhciBvcHRpb25zID0ge1xyXG4gICAgd2lkdGg6ICAnNjAwcHgnLFxyXG4gICAgaGVpZ2h0OiAnNjAwcHgnLFxyXG4gICAgc3R5bGU6ICdkb3Qtc2l6ZScsXHJcbiAgICBzaG93UGVyc3BlY3RpdmU6IHRydWUsXHJcbiAgICBzaG93TGVnZW5kOiB0cnVlLFxyXG4gICAgc2hvd0dyaWQ6IHRydWUsXHJcbiAgICBzaG93U2hhZG93OiBmYWxzZSxcclxuXHJcbiAgICAvLyBPcHRpb24gdG9vbHRpcCBjYW4gYmUgdHJ1ZSwgZmFsc2UsIG9yIGEgZnVuY3Rpb24gcmV0dXJuaW5nIGEgc3RyaW5nIHdpdGggSFRNTCBjb250ZW50c1xyXG4gICAgdG9vbHRpcDogZnVuY3Rpb24gKHBvaW50KSB7XHJcbiAgICAgIC8vIHBhcmFtZXRlciBwb2ludCBjb250YWlucyBwcm9wZXJ0aWVzIHgsIHksIHosIGFuZCBkYXRhXHJcbiAgICAgIC8vIGRhdGEgaXMgdGhlIG9yaWdpbmFsIG9iamVjdCBwYXNzZWQgdG8gdGhlIHBvaW50IGNvbnN0cnVjdG9yXHJcbiAgICAgIHJldHVybiAnc2NvcmU6IDxiPicgKyBwb2ludC56ICsgJzwvYj48YnI+JzsgLy8gKyBwb2ludC5kYXRhLmV4dHJhO1xyXG4gICAgfSxcclxuXHJcbiAgICAvLyBUb29sdGlwIGRlZmF1bHQgc3R5bGluZyBjYW4gYmUgb3ZlcnJpZGRlblxyXG4gICAgdG9vbHRpcFN0eWxlOiB7XHJcbiAgICAgIGNvbnRlbnQ6IHtcclxuICAgICAgICBiYWNrZ3JvdW5kICAgIDogJ3JnYmEoMjU1LCAyNTUsIDI1NSwgMC43KScsXHJcbiAgICAgICAgcGFkZGluZyAgICAgICA6ICcxMHB4JyxcclxuICAgICAgICBib3JkZXJSYWRpdXMgIDogJzEwcHgnXHJcbiAgICAgIH0sXHJcbiAgICAgIGxpbmU6IHtcclxuICAgICAgICBib3JkZXJMZWZ0ICAgIDogJzFweCBkb3R0ZWQgcmdiYSgwLCAwLCAwLCAwLjUpJ1xyXG4gICAgICB9LFxyXG4gICAgICBkb3Q6IHtcclxuICAgICAgICBib3JkZXIgICAgICAgIDogJzVweCBzb2xpZCByZ2JhKDAsIDAsIDAsIDAuNSknXHJcbiAgICAgIH1cclxuICAgIH0sXHJcblxyXG4gICAga2VlcEFzcGVjdFJhdGlvOiB0cnVlLFxyXG4gICAgdmVydGljYWxSYXRpbzogMC41XHJcbiAgfTtcclxuXHJcbiAgdmFyIGNhbWVyYSA9IGdyYXBoID8gZ3JhcGguZ2V0Q2FtZXJhUG9zaXRpb24oKSA6IG51bGw7XHJcblxyXG4gIC8vIGNyZWF0ZSBvdXIgZ3JhcGhcclxuICB2YXIgY29udGFpbmVyID0gZWxlbTtcclxuICBncmFwaCA9IG5ldyB2aXMuR3JhcGgzZChjb250YWluZXIsIGRhdGEsIG9wdGlvbnMpO1xyXG5cclxuICBpZiAoY2FtZXJhKSBncmFwaC5zZXRDYW1lcmFQb3NpdGlvbihjYW1lcmEpOyAvLyByZXN0b3JlIGNhbWVyYSBwb3NpdGlvblxyXG4gIHJldHVybiBncmFwaDtcclxufVxyXG4iLCJcclxubW9kdWxlLmV4cG9ydHMgPSBnZW5lcmF0ZVJhbmRvbTtcclxuZnVuY3Rpb24gZ2VuZXJhdGVSYW5kb20oKXtcclxuICByZXR1cm4gTWF0aC5yYW5kb20oKTtcclxufVxyXG4iLCIvLyBodHRwOi8vc3VubWluZ3Rhby5ibG9nc3BvdC5jb20vMjAxNi8xMS9pbmJyZWVkaW5nLWNvZWZmaWNpZW50Lmh0bWxcclxubW9kdWxlLmV4cG9ydHMgPSBnZXRJbmJyZWVkaW5nQ29lZmZpY2llbnQ7XHJcblxyXG5mdW5jdGlvbiBnZXRJbmJyZWVkaW5nQ29lZmZpY2llbnQoY2hpbGQpe1xyXG4gIHZhciBuYW1lSW5kZXggPSBuZXcgTWFwKCk7XHJcbiAgdmFyIGZsYWdnZWQgPSBuZXcgU2V0KCk7XHJcbiAgdmFyIGNvbnZlcmdlbmNlUG9pbnRzID0gbmV3IFNldCgpO1xyXG4gIGNyZWF0ZUFuY2VzdHJ5TWFwKGNoaWxkLCBbXSk7XHJcblxyXG4gIHZhciBzdG9yZWRDb2VmZmljaWVudHMgPSBuZXcgTWFwKCk7XHJcblxyXG4gIHJldHVybiBBcnJheS5mcm9tKGNvbnZlcmdlbmNlUG9pbnRzLnZhbHVlcygpKS5yZWR1Y2UoZnVuY3Rpb24oc3VtLCBwb2ludCl7XHJcbiAgICB2YXIgaUNvID0gZ2V0Q29lZmZpY2llbnQocG9pbnQpO1xyXG4gICAgcmV0dXJuIHN1bSArIGlDbztcclxuICB9LCAwKTtcclxuXHJcbiAgZnVuY3Rpb24gY3JlYXRlQW5jZXN0cnlNYXAoaW5pdE5vZGUpe1xyXG4gICAgdmFyIGl0ZW1zSW5RdWV1ZSA9IFt7IG5vZGU6IGluaXROb2RlLCBwYXRoOiBbXSB9XTtcclxuICAgIGRve1xyXG4gICAgICB2YXIgaXRlbSA9IGl0ZW1zSW5RdWV1ZS5zaGlmdCgpO1xyXG4gICAgICB2YXIgbm9kZSA9IGl0ZW0ubm9kZTtcclxuICAgICAgdmFyIHBhdGggPSBpdGVtLnBhdGg7XHJcbiAgICAgIGlmKHByb2Nlc3NJdGVtKG5vZGUsIHBhdGgpKXtcclxuICAgICAgICB2YXIgbmV4dFBhdGggPSBbIG5vZGUuaWQgXS5jb25jYXQocGF0aCk7XHJcbiAgICAgICAgaXRlbXNJblF1ZXVlID0gaXRlbXNJblF1ZXVlLmNvbmNhdChub2RlLmFuY2VzdHJ5Lm1hcChmdW5jdGlvbihwYXJlbnQpe1xyXG4gICAgICAgICAgcmV0dXJuIHtcclxuICAgICAgICAgICAgbm9kZTogcGFyZW50LFxyXG4gICAgICAgICAgICBwYXRoOiBuZXh0UGF0aFxyXG4gICAgICAgICAgfTtcclxuICAgICAgICB9KSk7XHJcbiAgICAgIH1cclxuICAgIH13aGlsZShpdGVtc0luUXVldWUubGVuZ3RoKTtcclxuXHJcblxyXG4gICAgZnVuY3Rpb24gcHJvY2Vzc0l0ZW0obm9kZSwgcGF0aCl7XHJcbiAgICAgIHZhciBuZXdBbmNlc3RvciA9ICFuYW1lSW5kZXguaGFzKG5vZGUuaWQpO1xyXG4gICAgICBpZihuZXdBbmNlc3Rvcil7XHJcbiAgICAgICAgbmFtZUluZGV4LnNldChub2RlLmlkLCB7XHJcbiAgICAgICAgICBwYXJlbnRzOiAobm9kZS5hbmNlc3RyeSB8fCBbXSkubWFwKGZ1bmN0aW9uKHBhcmVudCl7XHJcbiAgICAgICAgICAgIHJldHVybiBwYXJlbnQuaWQ7XHJcbiAgICAgICAgICB9KSxcclxuICAgICAgICAgIGlkOiBub2RlLmlkLFxyXG4gICAgICAgICAgY2hpbGRyZW46IFtdLFxyXG4gICAgICAgICAgY29udmVyZ2VuY2VzOiBbXSxcclxuICAgICAgICB9KTtcclxuICAgICAgfSBlbHNlIHtcclxuXHJcbiAgICAgICAgZmxhZ2dlZC5hZGQobm9kZS5pZClcclxuICAgICAgICBuYW1lSW5kZXguZ2V0KG5vZGUuaWQpLmNoaWxkcmVuLmZvckVhY2goZnVuY3Rpb24oY2hpbGRJZGVudGlmaWVyKXtcclxuICAgICAgICAgIHZhciBvZmZzZXRzID0gZmluZENvbnZlcmdlbmNlKGNoaWxkSWRlbnRpZmllci5wYXRoLCBwYXRoKTtcclxuICAgICAgICAgIGlmKCFvZmZzZXRzKXtcclxuICAgICAgICAgICAgcmV0dXJuO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgICAgdmFyIGNoaWxkSUQgPSBwYXRoW29mZnNldHNbMV1dO1xyXG4gICAgICAgICAgY29udmVyZ2VuY2VQb2ludHMuYWRkKGNoaWxkSUQpO1xyXG4gICAgICAgICAgbmFtZUluZGV4LmdldChjaGlsZElEKS5jb252ZXJnZW5jZXMucHVzaCh7XHJcbiAgICAgICAgICAgIHBhcmVudDogbm9kZS5pZCxcclxuICAgICAgICAgICAgb2Zmc2V0czogb2Zmc2V0cyxcclxuICAgICAgICAgIH0pO1xyXG4gICAgICAgIH0pO1xyXG4gICAgICB9XHJcblxyXG4gICAgICBpZihwYXRoLmxlbmd0aCl7XHJcbiAgICAgICAgbmFtZUluZGV4LmdldChub2RlLmlkKS5jaGlsZHJlbi5wdXNoKHtcclxuICAgICAgICAgIGNoaWxkOiBwYXRoWzBdLFxyXG4gICAgICAgICAgcGF0aDogcGF0aFxyXG4gICAgICAgIH0pO1xyXG4gICAgICB9XHJcblxyXG4gICAgICBpZighbmV3QW5jZXN0b3Ipe1xyXG4gICAgICAgIHJldHVybjtcclxuICAgICAgfVxyXG4gICAgICBpZighbm9kZS5hbmNlc3RyeSl7XHJcbiAgICAgICAgcmV0dXJuO1xyXG4gICAgICB9XHJcbiAgICAgIHJldHVybiB0cnVlO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgZnVuY3Rpb24gZ2V0Q29lZmZpY2llbnQoaWQpe1xyXG4gICAgaWYoc3RvcmVkQ29lZmZpY2llbnRzLmhhcyhpZCkpe1xyXG4gICAgICByZXR1cm4gc3RvcmVkQ29lZmZpY2llbnRzLmdldChpZCk7XHJcbiAgICB9XHJcbiAgICB2YXIgbm9kZSA9IG5hbWVJbmRleC5nZXQoaWQpO1xyXG4gICAgdmFyIHZhbCA9IG5vZGUuY29udmVyZ2VuY2VzLnJlZHVjZShmdW5jdGlvbihzdW0sIHBvaW50KXtcclxuICAgICAgcmV0dXJuIHN1bSArIE1hdGgucG93KDEgLyAyLCBwb2ludC5vZmZzZXRzLnJlZHVjZShmdW5jdGlvbihzdW0sIHZhbHVlKXtcclxuICAgICAgICByZXR1cm4gc3VtICsgdmFsdWU7XHJcbiAgICAgIH0sIDEpKSAqICgxICsgZ2V0Q29lZmZpY2llbnQocG9pbnQucGFyZW50KSk7XHJcbiAgICB9LCAwKTtcclxuICAgIHN0b3JlZENvZWZmaWNpZW50cy5zZXQoaWQsIHZhbCk7XHJcblxyXG4gICAgcmV0dXJuIHZhbDtcclxuXHJcbiAgfVxyXG4gIGZ1bmN0aW9uIGZpbmRDb252ZXJnZW5jZShsaXN0QSwgbGlzdEIpe1xyXG4gICAgdmFyIGNpLCBjaiwgbGksIGxqO1xyXG4gICAgb3V0ZXJsb29wOlxyXG4gICAgZm9yKGNpID0gMCwgbGkgPSBsaXN0QS5sZW5ndGg7IGNpIDwgbGk7IGNpKyspe1xyXG4gICAgICBmb3IoY2ogPSAwLCBsaiA9IGxpc3RCLmxlbmd0aDsgY2ogPCBsajsgY2orKyl7XHJcbiAgICAgICAgaWYobGlzdEFbY2ldID09PSBsaXN0Qltjal0pe1xyXG4gICAgICAgICAgYnJlYWsgb3V0ZXJsb29wO1xyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG4gICAgaWYoY2kgPT09IGxpKXtcclxuICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIFtjaSwgY2pdO1xyXG4gIH1cclxufVxyXG4iLCJ2YXIgY2FyQ29uc3RydWN0ID0gcmVxdWlyZShcIi4uL2Nhci1zY2hlbWEvY29uc3RydWN0LmpzXCIpO1xyXG5cclxudmFyIGNhckNvbnN0YW50cyA9IGNhckNvbnN0cnVjdC5jYXJDb25zdGFudHMoKTtcclxuXHJcbnZhciBzY2hlbWEgPSBjYXJDb25zdHJ1Y3QuZ2VuZXJhdGVTY2hlbWEoY2FyQ29uc3RhbnRzKTtcclxudmFyIHBpY2tQYXJlbnQgPSByZXF1aXJlKFwiLi9waWNrUGFyZW50XCIpO1xyXG52YXIgc2VsZWN0RnJvbUFsbFBhcmVudHMgPSByZXF1aXJlKFwiLi9zZWxlY3RGcm9tQWxsUGFyZW50c1wiKTtcclxuY29uc3QgY29uc3RhbnRzID0ge1xyXG4gIGdlbmVyYXRpb25TaXplOiAyMCxcclxuICBzY2hlbWE6IHNjaGVtYSxcclxuICBjaGFtcGlvbkxlbmd0aDogMSxcclxuICBtdXRhdGlvbl9yYW5nZTogMSxcclxuICBnZW5fbXV0YXRpb246IDAuMDUsXHJcbn07XHJcbm1vZHVsZS5leHBvcnRzID0gZnVuY3Rpb24oKXtcclxuICB2YXIgY3VycmVudENob2ljZXMgPSBuZXcgTWFwKCk7XHJcbiAgcmV0dXJuIE9iamVjdC5hc3NpZ24oXHJcbiAgICB7fSxcclxuICAgIGNvbnN0YW50cyxcclxuICAgIHtcclxuICAgICAgc2VsZWN0RnJvbUFsbFBhcmVudHM6IHNlbGVjdEZyb21BbGxQYXJlbnRzLFxyXG4gICAgICBnZW5lcmF0ZVJhbmRvbTogcmVxdWlyZShcIi4vZ2VuZXJhdGVSYW5kb21cIiksXHJcbiAgICAgIHBpY2tQYXJlbnQ6IHBpY2tQYXJlbnQuYmluZCh2b2lkIDAsIGN1cnJlbnRDaG9pY2VzKSxcclxuICAgIH1cclxuICApO1xyXG59XHJcbm1vZHVsZS5leHBvcnRzLmNvbnN0YW50cyA9IGNvbnN0YW50c1xyXG4iLCJ2YXIgbkF0dHJpYnV0ZXMgPSAxNTtcclxubW9kdWxlLmV4cG9ydHMgPSBwaWNrUGFyZW50O1xyXG5cclxuZnVuY3Rpb24gcGlja1BhcmVudChjdXJyZW50Q2hvaWNlcywgY2hvb3NlSWQsIGtleSAvKiAsIHBhcmVudHMgKi8pe1xyXG4gIGlmKCFjdXJyZW50Q2hvaWNlcy5oYXMoY2hvb3NlSWQpKXtcclxuICAgIGN1cnJlbnRDaG9pY2VzLnNldChjaG9vc2VJZCwgaW5pdGlhbGl6ZVBpY2soKSlcclxuICB9XHJcbiAgLy8gY29uc29sZS5sb2coY2hvb3NlSWQpO1xyXG4gIHZhciBzdGF0ZSA9IGN1cnJlbnRDaG9pY2VzLmdldChjaG9vc2VJZCk7XHJcbiAgLy8gY29uc29sZS5sb2coc3RhdGUuY3VycGFyZW50KTtcclxuICBzdGF0ZS5pKytcclxuICBpZihbXCJ3aGVlbF9yYWRpdXNcIiwgXCJ3aGVlbF92ZXJ0ZXhcIiwgXCJ3aGVlbF9kZW5zaXR5XCJdLmluZGV4T2Yoa2V5KSA+IC0xKXtcclxuICAgIHN0YXRlLmN1cnBhcmVudCA9IGN3X2Nob29zZVBhcmVudChzdGF0ZSk7XHJcbiAgICByZXR1cm4gc3RhdGUuY3VycGFyZW50O1xyXG4gIH1cclxuICBzdGF0ZS5jdXJwYXJlbnQgPSBjd19jaG9vc2VQYXJlbnQoc3RhdGUpO1xyXG4gIHJldHVybiBzdGF0ZS5jdXJwYXJlbnQ7XHJcblxyXG4gIGZ1bmN0aW9uIGN3X2Nob29zZVBhcmVudChzdGF0ZSkge1xyXG4gICAgdmFyIGN1cnBhcmVudCA9IHN0YXRlLmN1cnBhcmVudDtcclxuICAgIHZhciBhdHRyaWJ1dGVJbmRleCA9IHN0YXRlLmk7XHJcbiAgICB2YXIgc3dhcFBvaW50MSA9IHN0YXRlLnN3YXBQb2ludDFcclxuICAgIHZhciBzd2FwUG9pbnQyID0gc3RhdGUuc3dhcFBvaW50MlxyXG4gICAgLy8gY29uc29sZS5sb2coc3dhcFBvaW50MSwgc3dhcFBvaW50MiwgYXR0cmlidXRlSW5kZXgpXHJcbiAgICBpZiAoKHN3YXBQb2ludDEgPT0gYXR0cmlidXRlSW5kZXgpIHx8IChzd2FwUG9pbnQyID09IGF0dHJpYnV0ZUluZGV4KSkge1xyXG4gICAgICByZXR1cm4gY3VycGFyZW50ID09IDEgPyAwIDogMVxyXG4gICAgfVxyXG4gICAgcmV0dXJuIGN1cnBhcmVudFxyXG4gIH1cclxuXHJcbiAgZnVuY3Rpb24gaW5pdGlhbGl6ZVBpY2soKXtcclxuICAgIHZhciBjdXJwYXJlbnQgPSAwO1xyXG5cclxuICAgIHZhciBzd2FwUG9pbnQxID0gTWF0aC5mbG9vcihNYXRoLnJhbmRvbSgpICogKG5BdHRyaWJ1dGVzKSk7XHJcbiAgICB2YXIgc3dhcFBvaW50MiA9IHN3YXBQb2ludDE7XHJcbiAgICB3aGlsZSAoc3dhcFBvaW50MiA9PSBzd2FwUG9pbnQxKSB7XHJcbiAgICAgIHN3YXBQb2ludDIgPSBNYXRoLmZsb29yKE1hdGgucmFuZG9tKCkgKiAobkF0dHJpYnV0ZXMpKTtcclxuICAgIH1cclxuICAgIHZhciBpID0gMDtcclxuICAgIHJldHVybiB7XHJcbiAgICAgIGN1cnBhcmVudDogY3VycGFyZW50LFxyXG4gICAgICBpOiBpLFxyXG4gICAgICBzd2FwUG9pbnQxOiBzd2FwUG9pbnQxLFxyXG4gICAgICBzd2FwUG9pbnQyOiBzd2FwUG9pbnQyXHJcbiAgICB9XHJcbiAgfVxyXG59XHJcbiIsInZhciBnZXRJbmJyZWVkaW5nQ29lZmZpY2llbnQgPSByZXF1aXJlKFwiLi9pbmJyZWVkaW5nLWNvZWZmaWNpZW50XCIpO1xyXG5cclxubW9kdWxlLmV4cG9ydHMgPSBzaW1wbGVTZWxlY3Q7XHJcblxyXG5mdW5jdGlvbiBzaW1wbGVTZWxlY3QocGFyZW50cyl7XHJcbiAgdmFyIHRvdGFsUGFyZW50cyA9IHBhcmVudHMubGVuZ3RoXHJcbiAgdmFyIHIgPSBNYXRoLnJhbmRvbSgpO1xyXG4gIGlmIChyID09IDApXHJcbiAgICByZXR1cm4gMDtcclxuICByZXR1cm4gTWF0aC5mbG9vcigtTWF0aC5sb2cocikgKiB0b3RhbFBhcmVudHMpICUgdG90YWxQYXJlbnRzO1xyXG59XHJcblxyXG5mdW5jdGlvbiBzZWxlY3RGcm9tQWxsUGFyZW50cyhwYXJlbnRzLCBwYXJlbnRMaXN0LCBwcmV2aW91c1BhcmVudEluZGV4KSB7XHJcbiAgdmFyIHByZXZpb3VzUGFyZW50ID0gcGFyZW50c1twcmV2aW91c1BhcmVudEluZGV4XTtcclxuICB2YXIgdmFsaWRQYXJlbnRzID0gcGFyZW50cy5maWx0ZXIoZnVuY3Rpb24ocGFyZW50LCBpKXtcclxuICAgIGlmKHByZXZpb3VzUGFyZW50SW5kZXggPT09IGkpe1xyXG4gICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICB9XHJcbiAgICBpZighcHJldmlvdXNQYXJlbnQpe1xyXG4gICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH1cclxuICAgIHZhciBjaGlsZCA9IHtcclxuICAgICAgaWQ6IE1hdGgucmFuZG9tKCkudG9TdHJpbmcoMzIpLFxyXG4gICAgICBhbmNlc3RyeTogW3ByZXZpb3VzUGFyZW50LCBwYXJlbnRdLm1hcChmdW5jdGlvbihwKXtcclxuICAgICAgICByZXR1cm4ge1xyXG4gICAgICAgICAgaWQ6IHAuZGVmLmlkLFxyXG4gICAgICAgICAgYW5jZXN0cnk6IHAuZGVmLmFuY2VzdHJ5XHJcbiAgICAgICAgfVxyXG4gICAgICB9KVxyXG4gICAgfVxyXG4gICAgdmFyIGlDbyA9IGdldEluYnJlZWRpbmdDb2VmZmljaWVudChjaGlsZCk7XHJcbiAgICBjb25zb2xlLmxvZyhcImluYnJlZWRpbmcgY29lZmZpY2llbnRcIiwgaUNvKVxyXG4gICAgaWYoaUNvID4gMC4yNSl7XHJcbiAgICAgIHJldHVybiBmYWxzZTtcclxuICAgIH1cclxuICAgIHJldHVybiB0cnVlO1xyXG4gIH0pXHJcbiAgaWYodmFsaWRQYXJlbnRzLmxlbmd0aCA9PT0gMCl7XHJcbiAgICByZXR1cm4gTWF0aC5mbG9vcihNYXRoLnJhbmRvbSgpICogcGFyZW50cy5sZW5ndGgpXHJcbiAgfVxyXG4gIHZhciB0b3RhbFNjb3JlID0gdmFsaWRQYXJlbnRzLnJlZHVjZShmdW5jdGlvbihzdW0sIHBhcmVudCl7XHJcbiAgICByZXR1cm4gc3VtICsgcGFyZW50LnNjb3JlLnY7XHJcbiAgfSwgMCk7XHJcbiAgdmFyIHIgPSB0b3RhbFNjb3JlICogTWF0aC5yYW5kb20oKTtcclxuICBmb3IodmFyIGkgPSAwOyBpIDwgdmFsaWRQYXJlbnRzLmxlbmd0aDsgaSsrKXtcclxuICAgIHZhciBzY29yZSA9IHZhbGlkUGFyZW50c1tpXS5zY29yZS52O1xyXG4gICAgaWYociA+IHNjb3JlKXtcclxuICAgICAgciA9IHIgLSBzY29yZTtcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIGJyZWFrO1xyXG4gICAgfVxyXG4gIH1cclxuICByZXR1cm4gaTtcclxufVxyXG4iLCJcclxubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbihjYXIpIHtcclxuICB2YXIgb3V0ID0ge1xyXG4gICAgY2hhc3NpczogZ2hvc3RfZ2V0X2NoYXNzaXMoY2FyLmNoYXNzaXMpLFxyXG4gICAgd2hlZWxzOiBbXSxcclxuICAgIHBvczoge3g6IGNhci5jaGFzc2lzLkdldFBvc2l0aW9uKCkueCwgeTogY2FyLmNoYXNzaXMuR2V0UG9zaXRpb24oKS55fVxyXG4gIH07XHJcblxyXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgY2FyLndoZWVscy5sZW5ndGg7IGkrKykge1xyXG4gICAgb3V0LndoZWVsc1tpXSA9IGdob3N0X2dldF93aGVlbChjYXIud2hlZWxzW2ldKTtcclxuICB9XHJcblxyXG4gIHJldHVybiBvdXQ7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGdob3N0X2dldF9jaGFzc2lzKGMpIHtcclxuICB2YXIgZ2MgPSBbXTtcclxuXHJcbiAgZm9yICh2YXIgZiA9IGMuR2V0Rml4dHVyZUxpc3QoKTsgZjsgZiA9IGYubV9uZXh0KSB7XHJcbiAgICB2YXIgcyA9IGYuR2V0U2hhcGUoKTtcclxuXHJcbiAgICB2YXIgcCA9IHtcclxuICAgICAgdnR4OiBbXSxcclxuICAgICAgbnVtOiAwXHJcbiAgICB9XHJcblxyXG4gICAgcC5udW0gPSBzLm1fdmVydGV4Q291bnQ7XHJcblxyXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBzLm1fdmVydGV4Q291bnQ7IGkrKykge1xyXG4gICAgICBwLnZ0eC5wdXNoKGMuR2V0V29ybGRQb2ludChzLm1fdmVydGljZXNbaV0pKTtcclxuICAgIH1cclxuXHJcbiAgICBnYy5wdXNoKHApO1xyXG4gIH1cclxuXHJcbiAgcmV0dXJuIGdjO1xyXG59XHJcblxyXG5mdW5jdGlvbiBnaG9zdF9nZXRfd2hlZWwodykge1xyXG4gIHZhciBndyA9IFtdO1xyXG5cclxuICBmb3IgKHZhciBmID0gdy5HZXRGaXh0dXJlTGlzdCgpOyBmOyBmID0gZi5tX25leHQpIHtcclxuICAgIHZhciBzID0gZi5HZXRTaGFwZSgpO1xyXG5cclxuICAgIHZhciBjID0ge1xyXG4gICAgICBwb3M6IHcuR2V0V29ybGRQb2ludChzLm1fcCksXHJcbiAgICAgIHJhZDogcy5tX3JhZGl1cyxcclxuICAgICAgYW5nOiB3Lm1fc3dlZXAuYVxyXG4gICAgfVxyXG5cclxuICAgIGd3LnB1c2goYyk7XHJcbiAgfVxyXG5cclxuICByZXR1cm4gZ3c7XHJcbn1cclxuIiwiXHJcbnZhciBnaG9zdF9nZXRfZnJhbWUgPSByZXF1aXJlKFwiLi9jYXItdG8tZ2hvc3QuanNcIik7XHJcblxyXG52YXIgZW5hYmxlX2dob3N0ID0gdHJ1ZTtcclxuXHJcbm1vZHVsZS5leHBvcnRzID0ge1xyXG4gIGdob3N0X2NyZWF0ZV9yZXBsYXk6IGdob3N0X2NyZWF0ZV9yZXBsYXksXHJcbiAgZ2hvc3RfY3JlYXRlX2dob3N0OiBnaG9zdF9jcmVhdGVfZ2hvc3QsXHJcbiAgZ2hvc3RfcGF1c2U6IGdob3N0X3BhdXNlLFxyXG4gIGdob3N0X3Jlc3VtZTogZ2hvc3RfcmVzdW1lLFxyXG4gIGdob3N0X2dldF9wb3NpdGlvbjogZ2hvc3RfZ2V0X3Bvc2l0aW9uLFxyXG4gIGdob3N0X2NvbXBhcmVfdG9fcmVwbGF5OiBnaG9zdF9jb21wYXJlX3RvX3JlcGxheSxcclxuICBnaG9zdF9tb3ZlX2ZyYW1lOiBnaG9zdF9tb3ZlX2ZyYW1lLFxyXG4gIGdob3N0X2FkZF9yZXBsYXlfZnJhbWU6IGdob3N0X2FkZF9yZXBsYXlfZnJhbWUsXHJcbiAgZ2hvc3RfZHJhd19mcmFtZTogZ2hvc3RfZHJhd19mcmFtZSxcclxuICBnaG9zdF9yZXNldF9naG9zdDogZ2hvc3RfcmVzZXRfZ2hvc3RcclxufVxyXG5cclxuZnVuY3Rpb24gZ2hvc3RfY3JlYXRlX3JlcGxheSgpIHtcclxuICBpZiAoIWVuYWJsZV9naG9zdClcclxuICAgIHJldHVybiBudWxsO1xyXG5cclxuICByZXR1cm4ge1xyXG4gICAgbnVtX2ZyYW1lczogMCxcclxuICAgIGZyYW1lczogW10sXHJcbiAgfVxyXG59XHJcblxyXG5mdW5jdGlvbiBnaG9zdF9jcmVhdGVfZ2hvc3QoKSB7XHJcbiAgaWYgKCFlbmFibGVfZ2hvc3QpXHJcbiAgICByZXR1cm4gbnVsbDtcclxuXHJcbiAgcmV0dXJuIHtcclxuICAgIHJlcGxheTogbnVsbCxcclxuICAgIGZyYW1lOiAwLFxyXG4gICAgZGlzdDogLTEwMFxyXG4gIH1cclxufVxyXG5cclxuZnVuY3Rpb24gZ2hvc3RfcmVzZXRfZ2hvc3QoZ2hvc3QpIHtcclxuICBpZiAoIWVuYWJsZV9naG9zdClcclxuICAgIHJldHVybjtcclxuICBpZiAoZ2hvc3QgPT0gbnVsbClcclxuICAgIHJldHVybjtcclxuICBnaG9zdC5mcmFtZSA9IDA7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGdob3N0X3BhdXNlKGdob3N0KSB7XHJcbiAgaWYgKGdob3N0ICE9IG51bGwpXHJcbiAgICBnaG9zdC5vbGRfZnJhbWUgPSBnaG9zdC5mcmFtZTtcclxuICBnaG9zdF9yZXNldF9naG9zdChnaG9zdCk7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGdob3N0X3Jlc3VtZShnaG9zdCkge1xyXG4gIGlmIChnaG9zdCAhPSBudWxsKVxyXG4gICAgZ2hvc3QuZnJhbWUgPSBnaG9zdC5vbGRfZnJhbWU7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGdob3N0X2dldF9wb3NpdGlvbihnaG9zdCkge1xyXG4gIGlmICghZW5hYmxlX2dob3N0KVxyXG4gICAgcmV0dXJuO1xyXG4gIGlmIChnaG9zdCA9PSBudWxsKVxyXG4gICAgcmV0dXJuO1xyXG4gIGlmIChnaG9zdC5mcmFtZSA8IDApXHJcbiAgICByZXR1cm47XHJcbiAgaWYgKGdob3N0LnJlcGxheSA9PSBudWxsKVxyXG4gICAgcmV0dXJuO1xyXG4gIHZhciBmcmFtZSA9IGdob3N0LnJlcGxheS5mcmFtZXNbZ2hvc3QuZnJhbWVdO1xyXG4gIHJldHVybiBmcmFtZS5wb3M7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGdob3N0X2NvbXBhcmVfdG9fcmVwbGF5KHJlcGxheSwgZ2hvc3QsIG1heCkge1xyXG4gIGlmICghZW5hYmxlX2dob3N0KVxyXG4gICAgcmV0dXJuO1xyXG4gIGlmIChnaG9zdCA9PSBudWxsKVxyXG4gICAgcmV0dXJuO1xyXG4gIGlmIChyZXBsYXkgPT0gbnVsbClcclxuICAgIHJldHVybjtcclxuXHJcbiAgaWYgKGdob3N0LmRpc3QgPCBtYXgpIHtcclxuICAgIGdob3N0LnJlcGxheSA9IHJlcGxheTtcclxuICAgIGdob3N0LmRpc3QgPSBtYXg7XHJcbiAgICBnaG9zdC5mcmFtZSA9IDA7XHJcbiAgfVxyXG59XHJcblxyXG5mdW5jdGlvbiBnaG9zdF9tb3ZlX2ZyYW1lKGdob3N0KSB7XHJcbiAgaWYgKCFlbmFibGVfZ2hvc3QpXHJcbiAgICByZXR1cm47XHJcbiAgaWYgKGdob3N0ID09IG51bGwpXHJcbiAgICByZXR1cm47XHJcbiAgaWYgKGdob3N0LnJlcGxheSA9PSBudWxsKVxyXG4gICAgcmV0dXJuO1xyXG4gIGdob3N0LmZyYW1lKys7XHJcbiAgaWYgKGdob3N0LmZyYW1lID49IGdob3N0LnJlcGxheS5udW1fZnJhbWVzKVxyXG4gICAgZ2hvc3QuZnJhbWUgPSBnaG9zdC5yZXBsYXkubnVtX2ZyYW1lcyAtIDE7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGdob3N0X2FkZF9yZXBsYXlfZnJhbWUocmVwbGF5LCBjYXIpIHtcclxuICBpZiAoIWVuYWJsZV9naG9zdClcclxuICAgIHJldHVybjtcclxuICBpZiAocmVwbGF5ID09IG51bGwpXHJcbiAgICByZXR1cm47XHJcblxyXG4gIHZhciBmcmFtZSA9IGdob3N0X2dldF9mcmFtZShjYXIpO1xyXG4gIHJlcGxheS5mcmFtZXMucHVzaChmcmFtZSk7XHJcbiAgcmVwbGF5Lm51bV9mcmFtZXMrKztcclxufVxyXG5cclxuZnVuY3Rpb24gZ2hvc3RfZHJhd19mcmFtZShjdHgsIGdob3N0LCBjYW1lcmEpIHtcclxuICB2YXIgem9vbSA9IGNhbWVyYS56b29tO1xyXG4gIGlmICghZW5hYmxlX2dob3N0KVxyXG4gICAgcmV0dXJuO1xyXG4gIGlmIChnaG9zdCA9PSBudWxsKVxyXG4gICAgcmV0dXJuO1xyXG4gIGlmIChnaG9zdC5mcmFtZSA8IDApXHJcbiAgICByZXR1cm47XHJcbiAgaWYgKGdob3N0LnJlcGxheSA9PSBudWxsKVxyXG4gICAgcmV0dXJuO1xyXG5cclxuICB2YXIgZnJhbWUgPSBnaG9zdC5yZXBsYXkuZnJhbWVzW2dob3N0LmZyYW1lXTtcclxuXHJcbiAgLy8gd2hlZWwgc3R5bGVcclxuICBjdHguZmlsbFN0eWxlID0gXCIjZWVlXCI7XHJcbiAgY3R4LnN0cm9rZVN0eWxlID0gXCIjYWFhXCI7XHJcbiAgY3R4LmxpbmVXaWR0aCA9IDEgLyB6b29tO1xyXG5cclxuICBmb3IgKHZhciBpID0gMDsgaSA8IGZyYW1lLndoZWVscy5sZW5ndGg7IGkrKykge1xyXG4gICAgZm9yICh2YXIgdyBpbiBmcmFtZS53aGVlbHNbaV0pIHtcclxuICAgICAgZ2hvc3RfZHJhd19jaXJjbGUoY3R4LCBmcmFtZS53aGVlbHNbaV1bd10ucG9zLCBmcmFtZS53aGVlbHNbaV1bd10ucmFkLCBmcmFtZS53aGVlbHNbaV1bd10uYW5nKTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIC8vIGNoYXNzaXMgc3R5bGVcclxuICBjdHguc3Ryb2tlU3R5bGUgPSBcIiNhYWFcIjtcclxuICBjdHguZmlsbFN0eWxlID0gXCIjZWVlXCI7XHJcbiAgY3R4LmxpbmVXaWR0aCA9IDEgLyB6b29tO1xyXG4gIGN0eC5iZWdpblBhdGgoKTtcclxuICBmb3IgKHZhciBjIGluIGZyYW1lLmNoYXNzaXMpXHJcbiAgICBnaG9zdF9kcmF3X3BvbHkoY3R4LCBmcmFtZS5jaGFzc2lzW2NdLnZ0eCwgZnJhbWUuY2hhc3Npc1tjXS5udW0pO1xyXG4gIGN0eC5maWxsKCk7XHJcbiAgY3R4LnN0cm9rZSgpO1xyXG59XHJcblxyXG5mdW5jdGlvbiBnaG9zdF9kcmF3X3BvbHkoY3R4LCB2dHgsIG5fdnR4KSB7XHJcbiAgY3R4Lm1vdmVUbyh2dHhbMF0ueCwgdnR4WzBdLnkpO1xyXG4gIGZvciAodmFyIGkgPSAxOyBpIDwgbl92dHg7IGkrKykge1xyXG4gICAgY3R4LmxpbmVUbyh2dHhbaV0ueCwgdnR4W2ldLnkpO1xyXG4gIH1cclxuICBjdHgubGluZVRvKHZ0eFswXS54LCB2dHhbMF0ueSk7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGdob3N0X2RyYXdfY2lyY2xlKGN0eCwgY2VudGVyLCByYWRpdXMsIGFuZ2xlKSB7XHJcbiAgY3R4LmJlZ2luUGF0aCgpO1xyXG4gIGN0eC5hcmMoY2VudGVyLngsIGNlbnRlci55LCByYWRpdXMsIDAsIDIgKiBNYXRoLlBJLCB0cnVlKTtcclxuXHJcbiAgY3R4Lm1vdmVUbyhjZW50ZXIueCwgY2VudGVyLnkpO1xyXG4gIGN0eC5saW5lVG8oY2VudGVyLnggKyByYWRpdXMgKiBNYXRoLmNvcyhhbmdsZSksIGNlbnRlci55ICsgcmFkaXVzICogTWF0aC5zaW4oYW5nbGUpKTtcclxuXHJcbiAgY3R4LmZpbGwoKTtcclxuICBjdHguc3Ryb2tlKCk7XHJcbn1cclxuIiwiLyogZ2xvYmFscyBkb2N1bWVudCBwZXJmb3JtYW5jZSBsb2NhbFN0b3JhZ2UgYWxlcnQgY29uZmlybSBidG9hIEhUTUxEaXZFbGVtZW50ICovXHJcbi8qIGdsb2JhbHMgYjJWZWMyICovXHJcbi8vIEdsb2JhbCBWYXJzXHJcblxyXG52YXIgd29ybGRSdW4gPSByZXF1aXJlKFwiLi93b3JsZC9ydW4uanNcIik7XHJcbnZhciBjYXJDb25zdHJ1Y3QgPSByZXF1aXJlKFwiLi9jYXItc2NoZW1hL2NvbnN0cnVjdC5qc1wiKTtcclxuXHJcbnZhciBtYW5hZ2VSb3VuZCA9IHJlcXVpcmUoXCIuL21hY2hpbmUtbGVhcm5pbmcvZ2VuZXRpYy1hbGdvcml0aG0vbWFuYWdlLXJvdW5kLmpzXCIpO1xyXG5cclxudmFyIGdob3N0X2ZucyA9IHJlcXVpcmUoXCIuL2dob3N0L2luZGV4LmpzXCIpO1xyXG5cclxudmFyIGRyYXdDYXIgPSByZXF1aXJlKFwiLi9kcmF3L2RyYXctY2FyLmpzXCIpO1xyXG52YXIgZ3JhcGhfZm5zID0gcmVxdWlyZShcIi4vZHJhdy9wbG90LWdyYXBocy5qc1wiKTtcclxudmFyIHBsb3RfZ3JhcGhzID0gZ3JhcGhfZm5zLnBsb3RHcmFwaHM7XHJcbnZhciBjd19jbGVhckdyYXBoaWNzID0gZ3JhcGhfZm5zLmNsZWFyR3JhcGhpY3M7XHJcbnZhciBjd19kcmF3Rmxvb3IgPSByZXF1aXJlKFwiLi9kcmF3L2RyYXctZmxvb3IuanNcIik7XHJcblxyXG52YXIgZ2hvc3RfZHJhd19mcmFtZSA9IGdob3N0X2Zucy5naG9zdF9kcmF3X2ZyYW1lO1xyXG52YXIgZ2hvc3RfY3JlYXRlX2dob3N0ID0gZ2hvc3RfZm5zLmdob3N0X2NyZWF0ZV9naG9zdDtcclxudmFyIGdob3N0X2FkZF9yZXBsYXlfZnJhbWUgPSBnaG9zdF9mbnMuZ2hvc3RfYWRkX3JlcGxheV9mcmFtZTtcclxudmFyIGdob3N0X2NvbXBhcmVfdG9fcmVwbGF5ID0gZ2hvc3RfZm5zLmdob3N0X2NvbXBhcmVfdG9fcmVwbGF5O1xyXG52YXIgZ2hvc3RfZ2V0X3Bvc2l0aW9uID0gZ2hvc3RfZm5zLmdob3N0X2dldF9wb3NpdGlvbjtcclxudmFyIGdob3N0X21vdmVfZnJhbWUgPSBnaG9zdF9mbnMuZ2hvc3RfbW92ZV9mcmFtZTtcclxudmFyIGdob3N0X3Jlc2V0X2dob3N0ID0gZ2hvc3RfZm5zLmdob3N0X3Jlc2V0X2dob3N0XHJcbnZhciBnaG9zdF9wYXVzZSA9IGdob3N0X2Zucy5naG9zdF9wYXVzZTtcclxudmFyIGdob3N0X3Jlc3VtZSA9IGdob3N0X2Zucy5naG9zdF9yZXN1bWU7XHJcbnZhciBnaG9zdF9jcmVhdGVfcmVwbGF5ID0gZ2hvc3RfZm5zLmdob3N0X2NyZWF0ZV9yZXBsYXk7XHJcblxyXG52YXIgY3dfQ2FyID0gcmVxdWlyZShcIi4vZHJhdy9kcmF3LWNhci1zdGF0cy5qc1wiKTtcclxudmFyIGdob3N0O1xyXG52YXIgY2FyTWFwID0gbmV3IE1hcCgpO1xyXG5cclxudmFyIGRvRHJhdyA9IHRydWU7XHJcbnZhciBjd19wYXVzZWQgPSBmYWxzZTtcclxuXHJcbnZhciBib3gyZGZwcyA9IDYwO1xyXG52YXIgc2NyZWVuZnBzID0gNjA7XHJcbnZhciBza2lwVGlja3MgPSBNYXRoLnJvdW5kKDEwMDAgLyBib3gyZGZwcyk7XHJcbnZhciBtYXhGcmFtZVNraXAgPSBza2lwVGlja3MgKiAyO1xyXG5cclxudmFyIGNhbnZhcyA9IGRvY3VtZW50LmdldEVsZW1lbnRCeUlkKFwibWFpbmJveFwiKTtcclxudmFyIGN0eCA9IGNhbnZhcy5nZXRDb250ZXh0KFwiMmRcIik7XHJcblxyXG52YXIgY2FtZXJhID0ge1xyXG4gIHNwZWVkOiAwLjA1LFxyXG4gIHBvczoge1xyXG4gICAgeDogMCwgeTogMFxyXG4gIH0sXHJcbiAgdGFyZ2V0OiAtMSxcclxuICB6b29tOiA3MFxyXG59XHJcblxyXG52YXIgbWluaW1hcGNhbWVyYSA9IGRvY3VtZW50LmdldEVsZW1lbnRCeUlkKFwibWluaW1hcGNhbWVyYVwiKS5zdHlsZTtcclxudmFyIG1pbmltYXBob2xkZXIgPSBkb2N1bWVudC5xdWVyeVNlbGVjdG9yKFwiI21pbmltYXBob2xkZXJcIik7XHJcblxyXG52YXIgbWluaW1hcGNhbnZhcyA9IGRvY3VtZW50LmdldEVsZW1lbnRCeUlkKFwibWluaW1hcFwiKTtcclxudmFyIG1pbmltYXBjdHggPSBtaW5pbWFwY2FudmFzLmdldENvbnRleHQoXCIyZFwiKTtcclxudmFyIG1pbmltYXBzY2FsZSA9IDM7XHJcbnZhciBtaW5pbWFwZm9nZGlzdGFuY2UgPSAwO1xyXG52YXIgZm9nZGlzdGFuY2UgPSBkb2N1bWVudC5nZXRFbGVtZW50QnlJZChcIm1pbmltYXBmb2dcIikuc3R5bGU7XHJcblxyXG5cclxudmFyIGNhckNvbnN0YW50cyA9IGNhckNvbnN0cnVjdC5jYXJDb25zdGFudHMoKTtcclxuXHJcblxyXG52YXIgbWF4X2Nhcl9oZWFsdGggPSBib3gyZGZwcyAqIDEwO1xyXG5cclxudmFyIGN3X2dob3N0UmVwbGF5SW50ZXJ2YWwgPSBudWxsO1xyXG5cclxudmFyIGRpc3RhbmNlTWV0ZXIgPSBkb2N1bWVudC5nZXRFbGVtZW50QnlJZChcImRpc3RhbmNlbWV0ZXJcIik7XHJcbnZhciBoZWlnaHRNZXRlciA9IGRvY3VtZW50LmdldEVsZW1lbnRCeUlkKFwiaGVpZ2h0bWV0ZXJcIik7XHJcblxyXG52YXIgbGVhZGVyUG9zaXRpb24gPSB7XHJcbiAgeDogMCwgeTogMFxyXG59XHJcblxyXG5taW5pbWFwY2FtZXJhLndpZHRoID0gMTIgKiBtaW5pbWFwc2NhbGUgKyBcInB4XCI7XHJcbm1pbmltYXBjYW1lcmEuaGVpZ2h0ID0gNiAqIG1pbmltYXBzY2FsZSArIFwicHhcIjtcclxuXHJcblxyXG4vLyA9PT09PT09IFdPUkxEIFNUQVRFID09PT09PVxyXG52YXIgZ2VuZXJhdGlvbkNvbmZpZyA9IHJlcXVpcmUoXCIuL2dlbmVyYXRpb24tY29uZmlnXCIpO1xyXG5cclxuXHJcbnZhciB3b3JsZF9kZWYgPSB7XHJcbiAgZ3Jhdml0eTogbmV3IGIyVmVjMigwLjAsIC05LjgxKSxcclxuICBkb1NsZWVwOiB0cnVlLFxyXG4gIGZsb29yc2VlZDogYnRvYShNYXRoLnNlZWRyYW5kb20oKSksXHJcbiAgdGlsZURpbWVuc2lvbnM6IG5ldyBiMlZlYzIoMS41LCAwLjE1KSxcclxuICBtYXhGbG9vclRpbGVzOiAyMDAsXHJcbiAgbXV0YWJsZV9mbG9vcjogZmFsc2UsXHJcbiAgYm94MmRmcHM6IGJveDJkZnBzLFxyXG4gIG1vdG9yU3BlZWQ6IDgwLFxyXG4gIG1heF9jYXJfaGVhbHRoOiBtYXhfY2FyX2hlYWx0aCxcclxuICBzY2hlbWE6IGdlbmVyYXRpb25Db25maWcuY29uc3RhbnRzLnNjaGVtYVxyXG59XHJcblxyXG52YXIgY3dfZGVhZENhcnM7XHJcbnZhciBncmFwaFN0YXRlID0ge1xyXG4gIGN3X3RvcFNjb3JlczogW10sXHJcbiAgY3dfZ3JhcGhBdmVyYWdlOiBbXSxcclxuICBjd19ncmFwaEVsaXRlOiBbXSxcclxuICBjd19ncmFwaFRvcDogW10sXHJcbn07XHJcblxyXG5mdW5jdGlvbiByZXNldEdyYXBoU3RhdGUoKXtcclxuICBncmFwaFN0YXRlID0ge1xyXG4gICAgY3dfdG9wU2NvcmVzOiBbXSxcclxuICAgIGN3X2dyYXBoQXZlcmFnZTogW10sXHJcbiAgICBjd19ncmFwaEVsaXRlOiBbXSxcclxuICAgIGN3X2dyYXBoVG9wOiBbXSxcclxuICB9O1xyXG59XHJcblxyXG5cclxuXHJcbi8vID09PT09PT09PT09PT09PT09PT09PT09PT09XHJcblxyXG52YXIgZ2VuZXJhdGlvblN0YXRlO1xyXG5cclxuLy8gPT09PT09PT0gQWN0aXZpdHkgU3RhdGUgPT09PVxyXG52YXIgY3VycmVudFJ1bm5lcjtcclxudmFyIGxvb3BzID0gMDtcclxudmFyIG5leHRHYW1lVGljayA9IChuZXcgRGF0ZSkuZ2V0VGltZSgpO1xyXG5cclxuZnVuY3Rpb24gc2hvd0Rpc3RhbmNlKGRpc3RhbmNlLCBoZWlnaHQpIHtcclxuICBkaXN0YW5jZU1ldGVyLmlubmVySFRNTCA9IGRpc3RhbmNlICsgXCIgbWV0ZXJzPGJyIC8+XCI7XHJcbiAgaGVpZ2h0TWV0ZXIuaW5uZXJIVE1MID0gaGVpZ2h0ICsgXCIgbWV0ZXJzXCI7XHJcbiAgaWYgKGRpc3RhbmNlID4gbWluaW1hcGZvZ2Rpc3RhbmNlKSB7XHJcbiAgICBmb2dkaXN0YW5jZS53aWR0aCA9IDgwMCAtIE1hdGgucm91bmQoZGlzdGFuY2UgKyAxNSkgKiBtaW5pbWFwc2NhbGUgKyBcInB4XCI7XHJcbiAgICBtaW5pbWFwZm9nZGlzdGFuY2UgPSBkaXN0YW5jZTtcclxuICB9XHJcbn1cclxuXHJcblxyXG5cclxuLyogPT09IEVORCBDYXIgPT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PSAqL1xyXG4vKiA9PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09ICovXHJcblxyXG5cclxuLyogPT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PSAqL1xyXG4vKiA9PT09IEdlbmVyYXRpb24gPT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09ICovXHJcblxyXG5mdW5jdGlvbiBjd19nZW5lcmF0aW9uWmVybygpIHtcclxuXHJcbiAgZ2VuZXJhdGlvblN0YXRlID0gbWFuYWdlUm91bmQuZ2VuZXJhdGlvblplcm8oZ2VuZXJhdGlvbkNvbmZpZygpKTtcclxufVxyXG5cclxuZnVuY3Rpb24gcmVzZXRDYXJVSSgpe1xyXG4gIGN3X2RlYWRDYXJzID0gMDtcclxuICBsZWFkZXJQb3NpdGlvbiA9IHtcclxuICAgIHg6IDAsIHk6IDBcclxuICB9O1xyXG4gIGRvY3VtZW50LmdldEVsZW1lbnRCeUlkKFwiZ2VuZXJhdGlvblwiKS5pbm5lckhUTUwgPSBnZW5lcmF0aW9uU3RhdGUuY291bnRlci50b1N0cmluZygpO1xyXG4gIGRvY3VtZW50LmdldEVsZW1lbnRCeUlkKFwiY2Fyc1wiKS5pbm5lckhUTUwgPSBcIlwiO1xyXG4gIGRvY3VtZW50LmdldEVsZW1lbnRCeUlkKFwicG9wdWxhdGlvblwiKS5pbm5lckhUTUwgPSBnZW5lcmF0aW9uQ29uZmlnLmNvbnN0YW50cy5nZW5lcmF0aW9uU2l6ZS50b1N0cmluZygpO1xyXG59XHJcblxyXG4vKiA9PT09IEVORCBHZW5yYXRpb24gPT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09ICovXHJcbi8qID09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT0gKi9cclxuXHJcbi8qID09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT0gKi9cclxuLyogPT09PSBEcmF3aW5nID09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PSAqL1xyXG5cclxuZnVuY3Rpb24gY3dfZHJhd1NjcmVlbigpIHtcclxuICB2YXIgZmxvb3JUaWxlcyA9IGN1cnJlbnRSdW5uZXIuc2NlbmUuZmxvb3JUaWxlcztcclxuICBjdHguY2xlYXJSZWN0KDAsIDAsIGNhbnZhcy53aWR0aCwgY2FudmFzLmhlaWdodCk7XHJcbiAgY3R4LnNhdmUoKTtcclxuICBjd19zZXRDYW1lcmFQb3NpdGlvbigpO1xyXG4gIHZhciBjYW1lcmFfeCA9IGNhbWVyYS5wb3MueDtcclxuICB2YXIgY2FtZXJhX3kgPSBjYW1lcmEucG9zLnk7XHJcbiAgdmFyIHpvb20gPSBjYW1lcmEuem9vbTtcclxuICBjdHgudHJhbnNsYXRlKDIwMCAtIChjYW1lcmFfeCAqIHpvb20pLCAyMDAgKyAoY2FtZXJhX3kgKiB6b29tKSk7XHJcbiAgY3R4LnNjYWxlKHpvb20sIC16b29tKTtcclxuICBjd19kcmF3Rmxvb3IoY3R4LCBjYW1lcmEsIGZsb29yVGlsZXMpO1xyXG4gIGdob3N0X2RyYXdfZnJhbWUoY3R4LCBnaG9zdCwgY2FtZXJhKTtcclxuICBjd19kcmF3Q2FycygpO1xyXG4gIGN0eC5yZXN0b3JlKCk7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGN3X21pbmltYXBDYW1lcmEoLyogeCwgeSovKSB7XHJcbiAgdmFyIGNhbWVyYV94ID0gY2FtZXJhLnBvcy54XHJcbiAgdmFyIGNhbWVyYV95ID0gY2FtZXJhLnBvcy55XHJcbiAgbWluaW1hcGNhbWVyYS5sZWZ0ID0gTWF0aC5yb3VuZCgoMiArIGNhbWVyYV94KSAqIG1pbmltYXBzY2FsZSkgKyBcInB4XCI7XHJcbiAgbWluaW1hcGNhbWVyYS50b3AgPSBNYXRoLnJvdW5kKCgzMSAtIGNhbWVyYV95KSAqIG1pbmltYXBzY2FsZSkgKyBcInB4XCI7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGN3X3NldENhbWVyYVRhcmdldChrKSB7XHJcbiAgY2FtZXJhLnRhcmdldCA9IGs7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGN3X3NldENhbWVyYVBvc2l0aW9uKCkge1xyXG4gIHZhciBjYW1lcmFUYXJnZXRQb3NpdGlvblxyXG4gIGlmIChjYW1lcmEudGFyZ2V0ICE9PSAtMSkge1xyXG4gICAgY2FtZXJhVGFyZ2V0UG9zaXRpb24gPSBjYXJNYXAuZ2V0KGNhbWVyYS50YXJnZXQpLmdldFBvc2l0aW9uKCk7XHJcbiAgfSBlbHNlIHtcclxuICAgIGNhbWVyYVRhcmdldFBvc2l0aW9uID0gbGVhZGVyUG9zaXRpb247XHJcbiAgfVxyXG4gIHZhciBkaWZmX3kgPSBjYW1lcmEucG9zLnkgLSBjYW1lcmFUYXJnZXRQb3NpdGlvbi55O1xyXG4gIHZhciBkaWZmX3ggPSBjYW1lcmEucG9zLnggLSBjYW1lcmFUYXJnZXRQb3NpdGlvbi54O1xyXG4gIGNhbWVyYS5wb3MueSAtPSBjYW1lcmEuc3BlZWQgKiBkaWZmX3k7XHJcbiAgY2FtZXJhLnBvcy54IC09IGNhbWVyYS5zcGVlZCAqIGRpZmZfeDtcclxuICBjd19taW5pbWFwQ2FtZXJhKGNhbWVyYS5wb3MueCwgY2FtZXJhLnBvcy55KTtcclxufVxyXG5cclxuZnVuY3Rpb24gY3dfZHJhd0dob3N0UmVwbGF5KCkge1xyXG4gIHZhciBmbG9vclRpbGVzID0gY3VycmVudFJ1bm5lci5zY2VuZS5mbG9vclRpbGVzO1xyXG4gIHZhciBjYXJQb3NpdGlvbiA9IGdob3N0X2dldF9wb3NpdGlvbihnaG9zdCk7XHJcbiAgY2FtZXJhLnBvcy54ID0gY2FyUG9zaXRpb24ueDtcclxuICBjYW1lcmEucG9zLnkgPSBjYXJQb3NpdGlvbi55O1xyXG4gIGN3X21pbmltYXBDYW1lcmEoY2FtZXJhLnBvcy54LCBjYW1lcmEucG9zLnkpO1xyXG4gIHNob3dEaXN0YW5jZShcclxuICAgIE1hdGgucm91bmQoY2FyUG9zaXRpb24ueCAqIDEwMCkgLyAxMDAsXHJcbiAgICBNYXRoLnJvdW5kKGNhclBvc2l0aW9uLnkgKiAxMDApIC8gMTAwXHJcbiAgKTtcclxuICBjdHguY2xlYXJSZWN0KDAsIDAsIGNhbnZhcy53aWR0aCwgY2FudmFzLmhlaWdodCk7XHJcbiAgY3R4LnNhdmUoKTtcclxuICBjdHgudHJhbnNsYXRlKFxyXG4gICAgMjAwIC0gKGNhclBvc2l0aW9uLnggKiBjYW1lcmEuem9vbSksXHJcbiAgICAyMDAgKyAoY2FyUG9zaXRpb24ueSAqIGNhbWVyYS56b29tKVxyXG4gICk7XHJcbiAgY3R4LnNjYWxlKGNhbWVyYS56b29tLCAtY2FtZXJhLnpvb20pO1xyXG4gIGdob3N0X2RyYXdfZnJhbWUoY3R4LCBnaG9zdCk7XHJcbiAgZ2hvc3RfbW92ZV9mcmFtZShnaG9zdCk7XHJcbiAgY3dfZHJhd0Zsb29yKGN0eCwgY2FtZXJhLCBmbG9vclRpbGVzKTtcclxuICBjdHgucmVzdG9yZSgpO1xyXG59XHJcblxyXG5cclxuZnVuY3Rpb24gY3dfZHJhd0NhcnMoKSB7XHJcbiAgdmFyIGN3X2NhckFycmF5ID0gQXJyYXkuZnJvbShjYXJNYXAudmFsdWVzKCkpO1xyXG4gIGZvciAodmFyIGsgPSAoY3dfY2FyQXJyYXkubGVuZ3RoIC0gMSk7IGsgPj0gMDsgay0tKSB7XHJcbiAgICB2YXIgbXlDYXIgPSBjd19jYXJBcnJheVtrXTtcclxuICAgIGRyYXdDYXIoY2FyQ29uc3RhbnRzLCBteUNhciwgY2FtZXJhLCBjdHgpXHJcbiAgfVxyXG59XHJcblxyXG5mdW5jdGlvbiB0b2dnbGVEaXNwbGF5KCkge1xyXG4gIGNhbnZhcy53aWR0aCA9IGNhbnZhcy53aWR0aDtcclxuICBpZiAoZG9EcmF3KSB7XHJcbiAgICBkb0RyYXcgPSBmYWxzZTtcclxuICAgIGN3X3N0b3BTaW11bGF0aW9uKCk7XHJcbiAgICBjd19ydW5uaW5nSW50ZXJ2YWwgPSBzZXRJbnRlcnZhbChmdW5jdGlvbiAoKSB7XHJcbiAgICAgIHZhciB0aW1lID0gcGVyZm9ybWFuY2Uubm93KCkgKyAoMTAwMCAvIHNjcmVlbmZwcyk7XHJcbiAgICAgIHdoaWxlICh0aW1lID4gcGVyZm9ybWFuY2Uubm93KCkpIHtcclxuICAgICAgICBzaW11bGF0aW9uU3RlcCgpO1xyXG4gICAgICB9XHJcbiAgICB9LCAxKTtcclxuICB9IGVsc2Uge1xyXG4gICAgZG9EcmF3ID0gdHJ1ZTtcclxuICAgIGNsZWFySW50ZXJ2YWwoY3dfcnVubmluZ0ludGVydmFsKTtcclxuICAgIGN3X3N0YXJ0U2ltdWxhdGlvbigpO1xyXG4gIH1cclxufVxyXG5cclxuZnVuY3Rpb24gY3dfZHJhd01pbmlNYXAoKSB7XHJcbiAgdmFyIGZsb29yVGlsZXMgPSBjdXJyZW50UnVubmVyLnNjZW5lLmZsb29yVGlsZXM7XHJcbiAgdmFyIGxhc3RfdGlsZSA9IG51bGw7XHJcbiAgdmFyIHRpbGVfcG9zaXRpb24gPSBuZXcgYjJWZWMyKC01LCAwKTtcclxuICBtaW5pbWFwZm9nZGlzdGFuY2UgPSAwO1xyXG4gIGZvZ2Rpc3RhbmNlLndpZHRoID0gXCI4MDBweFwiO1xyXG4gIG1pbmltYXBjYW52YXMud2lkdGggPSBtaW5pbWFwY2FudmFzLndpZHRoO1xyXG4gIG1pbmltYXBjdHguc3Ryb2tlU3R5bGUgPSBcIiMzRjcyQUZcIjtcclxuICBtaW5pbWFwY3R4LmJlZ2luUGF0aCgpO1xyXG4gIG1pbmltYXBjdHgubW92ZVRvKDAsIDM1ICogbWluaW1hcHNjYWxlKTtcclxuICBmb3IgKHZhciBrID0gMDsgayA8IGZsb29yVGlsZXMubGVuZ3RoOyBrKyspIHtcclxuICAgIGxhc3RfdGlsZSA9IGZsb29yVGlsZXNba107XHJcbiAgICB2YXIgbGFzdF9maXh0dXJlID0gbGFzdF90aWxlLkdldEZpeHR1cmVMaXN0KCk7XHJcbiAgICB2YXIgbGFzdF93b3JsZF9jb29yZHMgPSBsYXN0X3RpbGUuR2V0V29ybGRQb2ludChsYXN0X2ZpeHR1cmUuR2V0U2hhcGUoKS5tX3ZlcnRpY2VzWzNdKTtcclxuICAgIHRpbGVfcG9zaXRpb24gPSBsYXN0X3dvcmxkX2Nvb3JkcztcclxuICAgIG1pbmltYXBjdHgubGluZVRvKCh0aWxlX3Bvc2l0aW9uLnggKyA1KSAqIG1pbmltYXBzY2FsZSwgKC10aWxlX3Bvc2l0aW9uLnkgKyAzNSkgKiBtaW5pbWFwc2NhbGUpO1xyXG4gIH1cclxuICBtaW5pbWFwY3R4LnN0cm9rZSgpO1xyXG59XHJcblxyXG4vKiA9PT09IEVORCBEcmF3aW5nID09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09ICovXHJcbi8qID09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT0gKi9cclxudmFyIHVpTGlzdGVuZXJzID0ge1xyXG4gIHByZUNhclN0ZXA6IGZ1bmN0aW9uKCl7XHJcbiAgICBnaG9zdF9tb3ZlX2ZyYW1lKGdob3N0KTtcclxuICB9LFxyXG4gIGNhclN0ZXAoY2FyKXtcclxuICAgIHVwZGF0ZUNhclVJKGNhcik7XHJcbiAgfSxcclxuICBjYXJEZWF0aChjYXJJbmZvKXtcclxuXHJcbiAgICB2YXIgayA9IGNhckluZm8uaW5kZXg7XHJcblxyXG4gICAgdmFyIGNhciA9IGNhckluZm8uY2FyLCBzY29yZSA9IGNhckluZm8uc2NvcmU7XHJcbiAgICBjYXJNYXAuZ2V0KGNhckluZm8pLmtpbGwoY3VycmVudFJ1bm5lciwgd29ybGRfZGVmKTtcclxuXHJcbiAgICAvLyByZWZvY3VzIGNhbWVyYSB0byBsZWFkZXIgb24gZGVhdGhcclxuICAgIGlmIChjYW1lcmEudGFyZ2V0ID09IGNhckluZm8pIHtcclxuICAgICAgY3dfc2V0Q2FtZXJhVGFyZ2V0KC0xKTtcclxuICAgIH1cclxuICAgIC8vIGNvbnNvbGUubG9nKHNjb3JlKTtcclxuICAgIGNhck1hcC5kZWxldGUoY2FySW5mbyk7XHJcbiAgICBnaG9zdF9jb21wYXJlX3RvX3JlcGxheShjYXIucmVwbGF5LCBnaG9zdCwgc2NvcmUudik7XHJcbiAgICBzY29yZS5pID0gZ2VuZXJhdGlvblN0YXRlLmNvdW50ZXI7XHJcblxyXG4gICAgY3dfZGVhZENhcnMrKztcclxuICAgIHZhciBnZW5lcmF0aW9uU2l6ZSA9IGdlbmVyYXRpb25Db25maWcuY29uc3RhbnRzLmdlbmVyYXRpb25TaXplO1xyXG4gICAgZG9jdW1lbnQuZ2V0RWxlbWVudEJ5SWQoXCJwb3B1bGF0aW9uXCIpLmlubmVySFRNTCA9IChnZW5lcmF0aW9uU2l6ZSAtIGN3X2RlYWRDYXJzKS50b1N0cmluZygpO1xyXG5cclxuICAgIC8vIGNvbnNvbGUubG9nKGxlYWRlclBvc2l0aW9uLmxlYWRlciwgaylcclxuICAgIGlmIChsZWFkZXJQb3NpdGlvbi5sZWFkZXIgPT0gaykge1xyXG4gICAgICAvLyBsZWFkZXIgaXMgZGVhZCwgZmluZCBuZXcgbGVhZGVyXHJcbiAgICAgIGN3X2ZpbmRMZWFkZXIoKTtcclxuICAgIH1cclxuICB9LFxyXG4gIGdlbmVyYXRpb25FbmQocmVzdWx0cyl7XHJcbiAgICBjbGVhbnVwUm91bmQocmVzdWx0cyk7XHJcbiAgICByZXR1cm4gY3dfbmV3Um91bmQocmVzdWx0cyk7XHJcbiAgfVxyXG59XHJcblxyXG5mdW5jdGlvbiBzaW11bGF0aW9uU3RlcCgpIHsgIFxyXG4gIGN1cnJlbnRSdW5uZXIuc3RlcCgpO1xyXG4gIHNob3dEaXN0YW5jZShcclxuICAgIE1hdGgucm91bmQobGVhZGVyUG9zaXRpb24ueCAqIDEwMCkgLyAxMDAsXHJcbiAgICBNYXRoLnJvdW5kKGxlYWRlclBvc2l0aW9uLnkgKiAxMDApIC8gMTAwXHJcbiAgKTtcclxufVxyXG5cclxuZnVuY3Rpb24gZ2FtZUxvb3AoKSB7XHJcbiAgbG9vcHMgPSAwO1xyXG4gIHdoaWxlICghY3dfcGF1c2VkICYmIChuZXcgRGF0ZSkuZ2V0VGltZSgpID4gbmV4dEdhbWVUaWNrICYmIGxvb3BzIDwgbWF4RnJhbWVTa2lwKSB7ICAgXHJcbiAgICBuZXh0R2FtZVRpY2sgKz0gc2tpcFRpY2tzO1xyXG4gICAgbG9vcHMrKztcclxuICB9XHJcbiAgc2ltdWxhdGlvblN0ZXAoKTtcclxuICBjd19kcmF3U2NyZWVuKCk7XHJcblxyXG4gIGlmKCFjd19wYXVzZWQpIHdpbmRvdy5yZXF1ZXN0QW5pbWF0aW9uRnJhbWUoZ2FtZUxvb3ApO1xyXG59XHJcblxyXG5mdW5jdGlvbiB1cGRhdGVDYXJVSShjYXJJbmZvKXtcclxuICB2YXIgayA9IGNhckluZm8uaW5kZXg7XHJcbiAgdmFyIGNhciA9IGNhck1hcC5nZXQoY2FySW5mbyk7XHJcbiAgdmFyIHBvc2l0aW9uID0gY2FyLmdldFBvc2l0aW9uKCk7XHJcblxyXG4gIGdob3N0X2FkZF9yZXBsYXlfZnJhbWUoY2FyLnJlcGxheSwgY2FyLmNhci5jYXIpO1xyXG4gIGNhci5taW5pbWFwbWFya2VyLnN0eWxlLmxlZnQgPSBNYXRoLnJvdW5kKChwb3NpdGlvbi54ICsgNSkgKiBtaW5pbWFwc2NhbGUpICsgXCJweFwiO1xyXG4gIGNhci5oZWFsdGhCYXIud2lkdGggPSBNYXRoLnJvdW5kKChjYXIuY2FyLnN0YXRlLmhlYWx0aCAvIG1heF9jYXJfaGVhbHRoKSAqIDEwMCkgKyBcIiVcIjtcclxuICBpZiAocG9zaXRpb24ueCA+IGxlYWRlclBvc2l0aW9uLngpIHtcclxuICAgIGxlYWRlclBvc2l0aW9uID0gcG9zaXRpb247XHJcbiAgICBsZWFkZXJQb3NpdGlvbi5sZWFkZXIgPSBrO1xyXG4gICAgLy8gY29uc29sZS5sb2coXCJuZXcgbGVhZGVyOiBcIiwgayk7XHJcbiAgfVxyXG59XHJcblxyXG5mdW5jdGlvbiBjd19maW5kTGVhZGVyKCkge1xyXG4gIHZhciBsZWFkID0gMDtcclxuICB2YXIgY3dfY2FyQXJyYXkgPSBBcnJheS5mcm9tKGNhck1hcC52YWx1ZXMoKSk7XHJcbiAgZm9yICh2YXIgayA9IDA7IGsgPCBjd19jYXJBcnJheS5sZW5ndGg7IGsrKykge1xyXG4gICAgaWYgKCFjd19jYXJBcnJheVtrXS5hbGl2ZSkge1xyXG4gICAgICBjb250aW51ZTtcclxuICAgIH1cclxuICAgIHZhciBwb3NpdGlvbiA9IGN3X2NhckFycmF5W2tdLmdldFBvc2l0aW9uKCk7XHJcbiAgICBpZiAocG9zaXRpb24ueCA+IGxlYWQpIHtcclxuICAgICAgbGVhZGVyUG9zaXRpb24gPSBwb3NpdGlvbjtcclxuICAgICAgbGVhZGVyUG9zaXRpb24ubGVhZGVyID0gaztcclxuICAgIH1cclxuICB9XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGZhc3RGb3J3YXJkKCl7XHJcbiAgdmFyIGdlbiA9IGdlbmVyYXRpb25TdGF0ZS5jb3VudGVyO1xyXG4gIHdoaWxlKGdlbiA9PT0gZ2VuZXJhdGlvblN0YXRlLmNvdW50ZXIpe1xyXG4gICAgY3VycmVudFJ1bm5lci5zdGVwKCk7XHJcbiAgfVxyXG59XHJcblxyXG5mdW5jdGlvbiBjbGVhbnVwUm91bmQocmVzdWx0cyl7XHJcblxyXG4gIHJlc3VsdHMuc29ydChmdW5jdGlvbiAoYSwgYikge1xyXG4gICAgaWYgKGEuc2NvcmUudiA+IGIuc2NvcmUudikge1xyXG4gICAgICByZXR1cm4gLTFcclxuICAgIH0gZWxzZSB7XHJcbiAgICAgIHJldHVybiAxXHJcbiAgICB9XHJcbiAgfSlcclxuICBncmFwaFN0YXRlID0gcGxvdF9ncmFwaHMoXHJcbiAgICBkb2N1bWVudC5nZXRFbGVtZW50QnlJZChcImdyYXBoY2FudmFzXCIpLFxyXG4gICAgZG9jdW1lbnQuZ2V0RWxlbWVudEJ5SWQoXCJ0b3BzY29yZXNcIiksXHJcbiAgICBudWxsLFxyXG4gICAgZ3JhcGhTdGF0ZSxcclxuICAgIHJlc3VsdHNcclxuICApO1xyXG59XHJcblxyXG5mdW5jdGlvbiBjd19uZXdSb3VuZChyZXN1bHRzKSB7XHJcbiAgY2FtZXJhLnBvcy54ID0gY2FtZXJhLnBvcy55ID0gMDtcclxuICBjd19zZXRDYW1lcmFUYXJnZXQoLTEpO1xyXG5cclxuICBnZW5lcmF0aW9uU3RhdGUgPSBtYW5hZ2VSb3VuZC5uZXh0R2VuZXJhdGlvbihcclxuICAgIGdlbmVyYXRpb25TdGF0ZSwgcmVzdWx0cywgZ2VuZXJhdGlvbkNvbmZpZygpXHJcbiAgKTtcclxuICBpZiAod29ybGRfZGVmLm11dGFibGVfZmxvb3IpIHtcclxuICAgIC8vIEdIT1NUIERJU0FCTEVEXHJcbiAgICBnaG9zdCA9IG51bGw7XHJcbiAgICB3b3JsZF9kZWYuZmxvb3JzZWVkID0gYnRvYShNYXRoLnNlZWRyYW5kb20oKSk7XHJcbiAgfSBlbHNlIHtcclxuICAgIC8vIFJFLUVOQUJMRSBHSE9TVFxyXG4gICAgZ2hvc3RfcmVzZXRfZ2hvc3QoZ2hvc3QpO1xyXG4gIH1cclxuICBjdXJyZW50UnVubmVyID0gd29ybGRSdW4od29ybGRfZGVmLCBnZW5lcmF0aW9uU3RhdGUuZ2VuZXJhdGlvbiwgdWlMaXN0ZW5lcnMpO1xyXG4gIHNldHVwQ2FyVUkoKTtcclxuICBjd19kcmF3TWluaU1hcCgpO1xyXG4gIHJlc2V0Q2FyVUkoKTtcclxufVxyXG5cclxuZnVuY3Rpb24gY3dfc3RhcnRTaW11bGF0aW9uKCkge1xyXG4gIGN3X3BhdXNlZCA9IGZhbHNlO1xyXG4gIHdpbmRvdy5yZXF1ZXN0QW5pbWF0aW9uRnJhbWUoZ2FtZUxvb3ApO1xyXG59XHJcblxyXG5mdW5jdGlvbiBjd19zdG9wU2ltdWxhdGlvbigpIHtcclxuICBjd19wYXVzZWQgPSB0cnVlO1xyXG59XHJcblxyXG5mdW5jdGlvbiBjd19jbGVhclBvcHVsYXRpb25Xb3JsZCgpIHtcclxuICBjYXJNYXAuZm9yRWFjaChmdW5jdGlvbihjYXIpe1xyXG4gICAgY2FyLmtpbGwoY3VycmVudFJ1bm5lciwgd29ybGRfZGVmKTtcclxuICB9KTtcclxufVxyXG5cclxuZnVuY3Rpb24gY3dfcmVzZXRQb3B1bGF0aW9uVUkoKSB7XHJcbiAgZG9jdW1lbnQuZ2V0RWxlbWVudEJ5SWQoXCJnZW5lcmF0aW9uXCIpLmlubmVySFRNTCA9IFwiXCI7XHJcbiAgZG9jdW1lbnQuZ2V0RWxlbWVudEJ5SWQoXCJjYXJzXCIpLmlubmVySFRNTCA9IFwiXCI7XHJcbiAgZG9jdW1lbnQuZ2V0RWxlbWVudEJ5SWQoXCJ0b3BzY29yZXNcIikuaW5uZXJIVE1MID0gXCJcIjtcclxuICBjd19jbGVhckdyYXBoaWNzKGRvY3VtZW50LmdldEVsZW1lbnRCeUlkKFwiZ3JhcGhjYW52YXNcIikpO1xyXG4gIHJlc2V0R3JhcGhTdGF0ZSgpO1xyXG59XHJcblxyXG5mdW5jdGlvbiBjd19yZXNldFdvcmxkKCkge1xyXG4gIGRvRHJhdyA9IHRydWU7XHJcbiAgY3dfc3RvcFNpbXVsYXRpb24oKTtcclxuICB3b3JsZF9kZWYuZmxvb3JzZWVkID0gZG9jdW1lbnQuZ2V0RWxlbWVudEJ5SWQoXCJuZXdzZWVkXCIpLnZhbHVlO1xyXG4gIGN3X2NsZWFyUG9wdWxhdGlvbldvcmxkKCk7XHJcbiAgY3dfcmVzZXRQb3B1bGF0aW9uVUkoKTtcclxuXHJcbiAgTWF0aC5zZWVkcmFuZG9tKCk7XHJcbiAgY3dfZ2VuZXJhdGlvblplcm8oKTtcclxuICBjdXJyZW50UnVubmVyID0gd29ybGRSdW4oXHJcbiAgICB3b3JsZF9kZWYsIGdlbmVyYXRpb25TdGF0ZS5nZW5lcmF0aW9uLCB1aUxpc3RlbmVyc1xyXG4gICk7XHJcblxyXG4gIGdob3N0ID0gZ2hvc3RfY3JlYXRlX2dob3N0KCk7XHJcbiAgcmVzZXRDYXJVSSgpO1xyXG4gIHNldHVwQ2FyVUkoKVxyXG4gIGN3X2RyYXdNaW5pTWFwKCk7XHJcblxyXG4gIGN3X3N0YXJ0U2ltdWxhdGlvbigpO1xyXG59XHJcblxyXG5mdW5jdGlvbiBzZXR1cENhclVJKCl7XHJcbiAgY3VycmVudFJ1bm5lci5jYXJzLm1hcChmdW5jdGlvbihjYXJJbmZvKXtcclxuICAgIHZhciBjYXIgPSBuZXcgY3dfQ2FyKGNhckluZm8sIGNhck1hcCk7XHJcbiAgICBjYXJNYXAuc2V0KGNhckluZm8sIGNhcik7XHJcbiAgICBjYXIucmVwbGF5ID0gZ2hvc3RfY3JlYXRlX3JlcGxheSgpO1xyXG4gICAgZ2hvc3RfYWRkX3JlcGxheV9mcmFtZShjYXIucmVwbGF5LCBjYXIuY2FyLmNhcik7XHJcbiAgfSlcclxufVxyXG5cclxuXHJcbmRvY3VtZW50LnF1ZXJ5U2VsZWN0b3IoXCIjZmFzdC1mb3J3YXJkXCIpLmFkZEV2ZW50TGlzdGVuZXIoXCJjbGlja1wiLCBmdW5jdGlvbigpe1xyXG4gIGZhc3RGb3J3YXJkKClcclxufSk7XHJcblxyXG5kb2N1bWVudC5xdWVyeVNlbGVjdG9yKFwiI3NhdmUtcHJvZ3Jlc3NcIikuYWRkRXZlbnRMaXN0ZW5lcihcImNsaWNrXCIsIGZ1bmN0aW9uKCl7XHJcbiAgc2F2ZVByb2dyZXNzKClcclxufSk7XHJcblxyXG5kb2N1bWVudC5xdWVyeVNlbGVjdG9yKFwiI3Jlc3RvcmUtcHJvZ3Jlc3NcIikuYWRkRXZlbnRMaXN0ZW5lcihcImNsaWNrXCIsIGZ1bmN0aW9uKCl7XHJcbiAgcmVzdG9yZVByb2dyZXNzKClcclxufSk7XHJcblxyXG5kb2N1bWVudC5xdWVyeVNlbGVjdG9yKFwiI3RvZ2dsZS1kaXNwbGF5XCIpLmFkZEV2ZW50TGlzdGVuZXIoXCJjbGlja1wiLCBmdW5jdGlvbigpe1xyXG4gIHRvZ2dsZURpc3BsYXkoKVxyXG59KVxyXG5cclxuZG9jdW1lbnQucXVlcnlTZWxlY3RvcihcIiNuZXctcG9wdWxhdGlvblwiKS5hZGRFdmVudExpc3RlbmVyKFwiY2xpY2tcIiwgZnVuY3Rpb24oKXtcclxuICBjd19yZXNldFBvcHVsYXRpb25VSSgpXHJcbiAgY3dfZ2VuZXJhdGlvblplcm8oKTtcclxuICBnaG9zdCA9IGdob3N0X2NyZWF0ZV9naG9zdCgpO1xyXG4gIHJlc2V0Q2FyVUkoKTtcclxufSlcclxuXHJcbmZ1bmN0aW9uIHNhdmVQcm9ncmVzcygpIHtcclxuICBsb2NhbFN0b3JhZ2UuY3dfc2F2ZWRHZW5lcmF0aW9uID0gSlNPTi5zdHJpbmdpZnkoZ2VuZXJhdGlvblN0YXRlLmdlbmVyYXRpb24pO1xyXG4gIGxvY2FsU3RvcmFnZS5jd19nZW5Db3VudGVyID0gZ2VuZXJhdGlvblN0YXRlLmNvdW50ZXI7XHJcbiAgbG9jYWxTdG9yYWdlLmN3X2dob3N0ID0gSlNPTi5zdHJpbmdpZnkoZ2hvc3QpO1xyXG4gIGxvY2FsU3RvcmFnZS5jd190b3BTY29yZXMgPSBKU09OLnN0cmluZ2lmeShncmFwaFN0YXRlLmN3X3RvcFNjb3Jlcyk7XHJcbiAgbG9jYWxTdG9yYWdlLmN3X2Zsb29yU2VlZCA9IHdvcmxkX2RlZi5mbG9vcnNlZWQ7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIHJlc3RvcmVQcm9ncmVzcygpIHtcclxuICBpZiAodHlwZW9mIGxvY2FsU3RvcmFnZS5jd19zYXZlZEdlbmVyYXRpb24gPT0gJ3VuZGVmaW5lZCcgfHwgbG9jYWxTdG9yYWdlLmN3X3NhdmVkR2VuZXJhdGlvbiA9PSBudWxsKSB7XHJcbiAgICBhbGVydChcIk5vIHNhdmVkIHByb2dyZXNzIGZvdW5kXCIpO1xyXG4gICAgcmV0dXJuO1xyXG4gIH1cclxuICBjd19zdG9wU2ltdWxhdGlvbigpO1xyXG4gIGdlbmVyYXRpb25TdGF0ZS5nZW5lcmF0aW9uID0gSlNPTi5wYXJzZShsb2NhbFN0b3JhZ2UuY3dfc2F2ZWRHZW5lcmF0aW9uKTtcclxuICBnZW5lcmF0aW9uU3RhdGUuY291bnRlciA9IGxvY2FsU3RvcmFnZS5jd19nZW5Db3VudGVyO1xyXG4gIGdob3N0ID0gSlNPTi5wYXJzZShsb2NhbFN0b3JhZ2UuY3dfZ2hvc3QpO1xyXG4gIGdyYXBoU3RhdGUuY3dfdG9wU2NvcmVzID0gSlNPTi5wYXJzZShsb2NhbFN0b3JhZ2UuY3dfdG9wU2NvcmVzKTtcclxuICB3b3JsZF9kZWYuZmxvb3JzZWVkID0gbG9jYWxTdG9yYWdlLmN3X2Zsb29yU2VlZDtcclxuICBkb2N1bWVudC5nZXRFbGVtZW50QnlJZChcIm5ld3NlZWRcIikudmFsdWUgPSB3b3JsZF9kZWYuZmxvb3JzZWVkO1xyXG5cclxuICBjdXJyZW50UnVubmVyID0gd29ybGRSdW4od29ybGRfZGVmLCBnZW5lcmF0aW9uU3RhdGUuZ2VuZXJhdGlvbiwgdWlMaXN0ZW5lcnMpO1xyXG4gIGN3X2RyYXdNaW5pTWFwKCk7XHJcbiAgTWF0aC5zZWVkcmFuZG9tKCk7XHJcblxyXG4gIHJlc2V0Q2FyVUkoKTtcclxuICBjd19zdGFydFNpbXVsYXRpb24oKTtcclxufVxyXG5cclxuZG9jdW1lbnQucXVlcnlTZWxlY3RvcihcIiNjb25maXJtLXJlc2V0XCIpLmFkZEV2ZW50TGlzdGVuZXIoXCJjbGlja1wiLCBmdW5jdGlvbigpe1xyXG4gIGN3X2NvbmZpcm1SZXNldFdvcmxkKClcclxufSlcclxuXHJcbmZ1bmN0aW9uIGN3X2NvbmZpcm1SZXNldFdvcmxkKCkge1xyXG4gIGlmIChjb25maXJtKCdSZWFsbHkgcmVzZXQgd29ybGQ/JykpIHtcclxuICAgIGN3X3Jlc2V0V29ybGQoKTtcclxuICB9IGVsc2Uge1xyXG4gICAgcmV0dXJuIGZhbHNlO1xyXG4gIH1cclxufVxyXG5cclxuLy8gZ2hvc3QgcmVwbGF5IHN0dWZmXHJcblxyXG5cclxuZnVuY3Rpb24gY3dfcGF1c2VTaW11bGF0aW9uKCkge1xyXG4gIGN3X3BhdXNlZCA9IHRydWU7XHJcbiAgZ2hvc3RfcGF1c2UoZ2hvc3QpO1xyXG59XHJcblxyXG5mdW5jdGlvbiBjd19yZXN1bWVTaW11bGF0aW9uKCkge1xyXG4gIGN3X3BhdXNlZCA9IGZhbHNlO1xyXG4gIGdob3N0X3Jlc3VtZShnaG9zdCk7XHJcbiAgd2luZG93LnJlcXVlc3RBbmltYXRpb25GcmFtZShnYW1lTG9vcCk7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGN3X3N0YXJ0R2hvc3RSZXBsYXkoKSB7XHJcbiAgaWYgKCFkb0RyYXcpIHtcclxuICAgIHRvZ2dsZURpc3BsYXkoKTtcclxuICB9XHJcbiAgY3dfcGF1c2VTaW11bGF0aW9uKCk7XHJcbiAgY3dfZ2hvc3RSZXBsYXlJbnRlcnZhbCA9IHNldEludGVydmFsKGN3X2RyYXdHaG9zdFJlcGxheSwgTWF0aC5yb3VuZCgxMDAwIC8gc2NyZWVuZnBzKSk7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGN3X3N0b3BHaG9zdFJlcGxheSgpIHtcclxuICBjbGVhckludGVydmFsKGN3X2dob3N0UmVwbGF5SW50ZXJ2YWwpO1xyXG4gIGN3X2dob3N0UmVwbGF5SW50ZXJ2YWwgPSBudWxsO1xyXG4gIGN3X2ZpbmRMZWFkZXIoKTtcclxuICBjYW1lcmEucG9zLnggPSBsZWFkZXJQb3NpdGlvbi54O1xyXG4gIGNhbWVyYS5wb3MueSA9IGxlYWRlclBvc2l0aW9uLnk7XHJcbiAgY3dfcmVzdW1lU2ltdWxhdGlvbigpO1xyXG59XHJcblxyXG5kb2N1bWVudC5xdWVyeVNlbGVjdG9yKFwiI3RvZ2dsZS1naG9zdFwiKS5hZGRFdmVudExpc3RlbmVyKFwiY2xpY2tcIiwgZnVuY3Rpb24oZSl7XHJcbiAgY3dfdG9nZ2xlR2hvc3RSZXBsYXkoZS50YXJnZXQpXHJcbn0pXHJcblxyXG5mdW5jdGlvbiBjd190b2dnbGVHaG9zdFJlcGxheShidXR0b24pIHtcclxuICBpZiAoY3dfZ2hvc3RSZXBsYXlJbnRlcnZhbCA9PSBudWxsKSB7XHJcbiAgICBjd19zdGFydEdob3N0UmVwbGF5KCk7XHJcbiAgICBidXR0b24udmFsdWUgPSBcIlJlc3VtZSBzaW11bGF0aW9uXCI7XHJcbiAgfSBlbHNlIHtcclxuICAgIGN3X3N0b3BHaG9zdFJlcGxheSgpO1xyXG4gICAgYnV0dG9uLnZhbHVlID0gXCJWaWV3IHRvcCByZXBsYXlcIjtcclxuICB9XHJcbn1cclxuLy8gZ2hvc3QgcmVwbGF5IHN0dWZmIEVORFxyXG5cclxuLy8gaW5pdGlhbCBzdHVmZiwgb25seSBjYWxsZWQgb25jZSAoaG9wZWZ1bGx5KVxyXG5mdW5jdGlvbiBjd19pbml0KCkge1xyXG4gIC8vIGNsb25lIHNpbHZlciBkb3QgYW5kIGhlYWx0aCBiYXJcclxuICB2YXIgbW1tID0gZG9jdW1lbnQuZ2V0RWxlbWVudHNCeU5hbWUoJ21pbmltYXBtYXJrZXInKVswXTtcclxuICB2YXIgaGJhciA9IGRvY3VtZW50LmdldEVsZW1lbnRzQnlOYW1lKCdoZWFsdGhiYXInKVswXTtcclxuICB2YXIgZ2VuZXJhdGlvblNpemUgPSBnZW5lcmF0aW9uQ29uZmlnLmNvbnN0YW50cy5nZW5lcmF0aW9uU2l6ZTtcclxuXHJcbiAgZm9yICh2YXIgayA9IDA7IGsgPCBnZW5lcmF0aW9uU2l6ZTsgaysrKSB7XHJcblxyXG4gICAgLy8gbWluaW1hcCBtYXJrZXJzXHJcbiAgICB2YXIgbmV3YmFyID0gbW1tLmNsb25lTm9kZSh0cnVlKTtcclxuICAgIG5ld2Jhci5pZCA9IFwiYmFyXCIgKyBrO1xyXG4gICAgbmV3YmFyLnN0eWxlLnBhZGRpbmdUb3AgPSBrICogOSArIFwicHhcIjtcclxuICAgIG1pbmltYXBob2xkZXIuYXBwZW5kQ2hpbGQobmV3YmFyKTtcclxuXHJcbiAgICAvLyBoZWFsdGggYmFyc1xyXG4gICAgdmFyIG5ld2hlYWx0aCA9IGhiYXIuY2xvbmVOb2RlKHRydWUpO1xyXG4gICAgbmV3aGVhbHRoLmdldEVsZW1lbnRzQnlUYWdOYW1lKFwiRElWXCIpWzBdLmlkID0gXCJoZWFsdGhcIiArIGs7XHJcbiAgICBuZXdoZWFsdGguY2FyX2luZGV4ID0gaztcclxuICAgIGRvY3VtZW50LmdldEVsZW1lbnRCeUlkKFwiaGVhbHRoXCIpLmFwcGVuZENoaWxkKG5ld2hlYWx0aCk7XHJcbiAgfVxyXG4gIG1tbS5wYXJlbnROb2RlLnJlbW92ZUNoaWxkKG1tbSk7XHJcbiAgaGJhci5wYXJlbnROb2RlLnJlbW92ZUNoaWxkKGhiYXIpO1xyXG4gIHdvcmxkX2RlZi5mbG9vcnNlZWQgPSBidG9hKE1hdGguc2VlZHJhbmRvbSgpKTtcclxuICBjd19nZW5lcmF0aW9uWmVybygpO1xyXG4gIGdob3N0ID0gZ2hvc3RfY3JlYXRlX2dob3N0KCk7XHJcbiAgcmVzZXRDYXJVSSgpO1xyXG4gIGN1cnJlbnRSdW5uZXIgPSB3b3JsZFJ1bih3b3JsZF9kZWYsIGdlbmVyYXRpb25TdGF0ZS5nZW5lcmF0aW9uLCB1aUxpc3RlbmVycyk7XHJcbiAgc2V0dXBDYXJVSSgpO1xyXG4gIGN3X2RyYXdNaW5pTWFwKCk7XHJcbiAgd2luZG93LnJlcXVlc3RBbmltYXRpb25GcmFtZShnYW1lTG9vcCk7XHJcbiAgXHJcbn1cclxuXHJcbmZ1bmN0aW9uIHJlbE1vdXNlQ29vcmRzKGV2ZW50KSB7XHJcbiAgdmFyIHRvdGFsT2Zmc2V0WCA9IDA7XHJcbiAgdmFyIHRvdGFsT2Zmc2V0WSA9IDA7XHJcbiAgdmFyIGNhbnZhc1ggPSAwO1xyXG4gIHZhciBjYW52YXNZID0gMDtcclxuICB2YXIgY3VycmVudEVsZW1lbnQgPSB0aGlzO1xyXG5cclxuICBkbyB7XHJcbiAgICB0b3RhbE9mZnNldFggKz0gY3VycmVudEVsZW1lbnQub2Zmc2V0TGVmdCAtIGN1cnJlbnRFbGVtZW50LnNjcm9sbExlZnQ7XHJcbiAgICB0b3RhbE9mZnNldFkgKz0gY3VycmVudEVsZW1lbnQub2Zmc2V0VG9wIC0gY3VycmVudEVsZW1lbnQuc2Nyb2xsVG9wO1xyXG4gICAgY3VycmVudEVsZW1lbnQgPSBjdXJyZW50RWxlbWVudC5vZmZzZXRQYXJlbnRcclxuICB9XHJcbiAgd2hpbGUgKGN1cnJlbnRFbGVtZW50KTtcclxuXHJcbiAgY2FudmFzWCA9IGV2ZW50LnBhZ2VYIC0gdG90YWxPZmZzZXRYO1xyXG4gIGNhbnZhc1kgPSBldmVudC5wYWdlWSAtIHRvdGFsT2Zmc2V0WTtcclxuXHJcbiAgcmV0dXJuIHt4OiBjYW52YXNYLCB5OiBjYW52YXNZfVxyXG59XHJcbkhUTUxEaXZFbGVtZW50LnByb3RvdHlwZS5yZWxNb3VzZUNvb3JkcyA9IHJlbE1vdXNlQ29vcmRzO1xyXG5taW5pbWFwaG9sZGVyLm9uY2xpY2sgPSBmdW5jdGlvbiAoZXZlbnQpIHtcclxuICB2YXIgY29vcmRzID0gbWluaW1hcGhvbGRlci5yZWxNb3VzZUNvb3JkcyhldmVudCk7XHJcbiAgdmFyIGN3X2NhckFycmF5ID0gQXJyYXkuZnJvbShjYXJNYXAudmFsdWVzKCkpO1xyXG4gIHZhciBjbG9zZXN0ID0ge1xyXG4gICAgdmFsdWU6IGN3X2NhckFycmF5WzBdLmNhcixcclxuICAgIGRpc3Q6IE1hdGguYWJzKCgoY3dfY2FyQXJyYXlbMF0uZ2V0UG9zaXRpb24oKS54ICsgNikgKiBtaW5pbWFwc2NhbGUpIC0gY29vcmRzLngpLFxyXG4gICAgeDogY3dfY2FyQXJyYXlbMF0uZ2V0UG9zaXRpb24oKS54XHJcbiAgfVxyXG5cclxuICB2YXIgbWF4WCA9IDA7XHJcbiAgZm9yICh2YXIgaSA9IDA7IGkgPCBjd19jYXJBcnJheS5sZW5ndGg7IGkrKykge1xyXG4gICAgdmFyIHBvcyA9IGN3X2NhckFycmF5W2ldLmdldFBvc2l0aW9uKCk7XHJcbiAgICB2YXIgZGlzdCA9IE1hdGguYWJzKCgocG9zLnggKyA2KSAqIG1pbmltYXBzY2FsZSkgLSBjb29yZHMueCk7XHJcbiAgICBpZiAoZGlzdCA8IGNsb3Nlc3QuZGlzdCkge1xyXG4gICAgICBjbG9zZXN0LnZhbHVlID0gY3dfY2FyQXJyYXkuY2FyO1xyXG4gICAgICBjbG9zZXN0LmRpc3QgPSBkaXN0O1xyXG4gICAgICBjbG9zZXN0LnggPSBwb3MueDtcclxuICAgIH1cclxuICAgIG1heFggPSBNYXRoLm1heChwb3MueCwgbWF4WCk7XHJcbiAgfVxyXG5cclxuICBpZiAoY2xvc2VzdC54ID09IG1heFgpIHsgLy8gZm9jdXMgb24gbGVhZGVyIGFnYWluXHJcbiAgICBjd19zZXRDYW1lcmFUYXJnZXQoLTEpO1xyXG4gIH0gZWxzZSB7XHJcbiAgICBjd19zZXRDYW1lcmFUYXJnZXQoY2xvc2VzdC52YWx1ZSk7XHJcbiAgfVxyXG59XHJcblxyXG5cclxuZG9jdW1lbnQucXVlcnlTZWxlY3RvcihcIiNtdXRhdGlvbnJhdGVcIikuYWRkRXZlbnRMaXN0ZW5lcihcImNoYW5nZVwiLCBmdW5jdGlvbihlKXtcclxuICB2YXIgZWxlbSA9IGUudGFyZ2V0XHJcbiAgY3dfc2V0TXV0YXRpb24oZWxlbS5vcHRpb25zW2VsZW0uc2VsZWN0ZWRJbmRleF0udmFsdWUpXHJcbn0pXHJcblxyXG5kb2N1bWVudC5xdWVyeVNlbGVjdG9yKFwiI211dGF0aW9uc2l6ZVwiKS5hZGRFdmVudExpc3RlbmVyKFwiY2hhbmdlXCIsIGZ1bmN0aW9uKGUpe1xyXG4gIHZhciBlbGVtID0gZS50YXJnZXRcclxuICBjd19zZXRNdXRhdGlvblJhbmdlKGVsZW0ub3B0aW9uc1tlbGVtLnNlbGVjdGVkSW5kZXhdLnZhbHVlKVxyXG59KVxyXG5cclxuZG9jdW1lbnQucXVlcnlTZWxlY3RvcihcIiNmbG9vclwiKS5hZGRFdmVudExpc3RlbmVyKFwiY2hhbmdlXCIsIGZ1bmN0aW9uKGUpe1xyXG4gIHZhciBlbGVtID0gZS50YXJnZXRcclxuICBjd19zZXRNdXRhYmxlRmxvb3IoZWxlbS5vcHRpb25zW2VsZW0uc2VsZWN0ZWRJbmRleF0udmFsdWUpXHJcbn0pO1xyXG5cclxuZG9jdW1lbnQucXVlcnlTZWxlY3RvcihcIiNncmF2aXR5XCIpLmFkZEV2ZW50TGlzdGVuZXIoXCJjaGFuZ2VcIiwgZnVuY3Rpb24oZSl7XHJcbiAgdmFyIGVsZW0gPSBlLnRhcmdldFxyXG4gIGN3X3NldEdyYXZpdHkoZWxlbS5vcHRpb25zW2VsZW0uc2VsZWN0ZWRJbmRleF0udmFsdWUpXHJcbn0pXHJcblxyXG5kb2N1bWVudC5xdWVyeVNlbGVjdG9yKFwiI2VsaXRlc2l6ZVwiKS5hZGRFdmVudExpc3RlbmVyKFwiY2hhbmdlXCIsIGZ1bmN0aW9uKGUpe1xyXG4gIHZhciBlbGVtID0gZS50YXJnZXRcclxuICBjd19zZXRFbGl0ZVNpemUoZWxlbS5vcHRpb25zW2VsZW0uc2VsZWN0ZWRJbmRleF0udmFsdWUpXHJcbn0pXHJcblxyXG5mdW5jdGlvbiBjd19zZXRNdXRhdGlvbihtdXRhdGlvbikge1xyXG4gIGdlbmVyYXRpb25Db25maWcuY29uc3RhbnRzLmdlbl9tdXRhdGlvbiA9IHBhcnNlRmxvYXQobXV0YXRpb24pO1xyXG59XHJcblxyXG5mdW5jdGlvbiBjd19zZXRNdXRhdGlvblJhbmdlKHJhbmdlKSB7XHJcbiAgZ2VuZXJhdGlvbkNvbmZpZy5jb25zdGFudHMubXV0YXRpb25fcmFuZ2UgPSBwYXJzZUZsb2F0KHJhbmdlKTtcclxufVxyXG5cclxuZnVuY3Rpb24gY3dfc2V0TXV0YWJsZUZsb29yKGNob2ljZSkge1xyXG4gIHdvcmxkX2RlZi5tdXRhYmxlX2Zsb29yID0gKGNob2ljZSA9PSAxKTtcclxufVxyXG5cclxuZnVuY3Rpb24gY3dfc2V0R3Jhdml0eShjaG9pY2UpIHtcclxuICB3b3JsZF9kZWYuZ3Jhdml0eSA9IG5ldyBiMlZlYzIoMC4wLCAtcGFyc2VGbG9hdChjaG9pY2UpKTtcclxuICB2YXIgd29ybGQgPSBjdXJyZW50UnVubmVyLnNjZW5lLndvcmxkXHJcbiAgLy8gQ0hFQ0sgR1JBVklUWSBDSEFOR0VTXHJcbiAgaWYgKHdvcmxkLkdldEdyYXZpdHkoKS55ICE9IHdvcmxkX2RlZi5ncmF2aXR5LnkpIHtcclxuICAgIHdvcmxkLlNldEdyYXZpdHkod29ybGRfZGVmLmdyYXZpdHkpO1xyXG4gIH1cclxufVxyXG5cclxuZnVuY3Rpb24gY3dfc2V0RWxpdGVTaXplKGNsb25lcykge1xyXG4gIGdlbmVyYXRpb25Db25maWcuY29uc3RhbnRzLmNoYW1waW9uTGVuZ3RoID0gcGFyc2VJbnQoY2xvbmVzLCAxMCk7XHJcbn1cclxuXHJcbmN3X2luaXQoKTtcclxuIiwidmFyIHJhbmRvbSA9IHJlcXVpcmUoXCIuL3JhbmRvbS5qc1wiKTtcclxuXHJcbm1vZHVsZS5leHBvcnRzID0ge1xyXG4gIGNyZWF0ZUdlbmVyYXRpb25aZXJvKHNjaGVtYSwgZ2VuZXJhdG9yKXtcclxuICAgIHJldHVybiBPYmplY3Qua2V5cyhzY2hlbWEpLnJlZHVjZShmdW5jdGlvbihpbnN0YW5jZSwga2V5KXtcclxuICAgICAgdmFyIHNjaGVtYVByb3AgPSBzY2hlbWFba2V5XTtcclxuICAgICAgdmFyIHZhbHVlcyA9IHJhbmRvbS5jcmVhdGVOb3JtYWxzKHNjaGVtYVByb3AsIGdlbmVyYXRvcik7XHJcbiAgICAgIGluc3RhbmNlW2tleV0gPSB2YWx1ZXM7XHJcbiAgICAgIHJldHVybiBpbnN0YW5jZTtcclxuICAgIH0sIHsgaWQ6IE1hdGgucmFuZG9tKCkudG9TdHJpbmcoMzIpIH0pO1xyXG4gIH0sXHJcbiAgY3JlYXRlQ3Jvc3NCcmVlZChzY2hlbWEsIHBhcmVudHMsIHBhcmVudENob29zZXIpe1xyXG4gICAgdmFyIGlkID0gTWF0aC5yYW5kb20oKS50b1N0cmluZygzMik7XHJcbiAgICByZXR1cm4gT2JqZWN0LmtleXMoc2NoZW1hKS5yZWR1Y2UoZnVuY3Rpb24oY3Jvc3NEZWYsIGtleSl7XHJcbiAgICAgIHZhciBzY2hlbWFEZWYgPSBzY2hlbWFba2V5XTtcclxuICAgICAgdmFyIHZhbHVlcyA9IFtdO1xyXG4gICAgICBmb3IodmFyIGkgPSAwLCBsID0gc2NoZW1hRGVmLmxlbmd0aDsgaSA8IGw7IGkrKyl7XHJcbiAgICAgICAgdmFyIHAgPSBwYXJlbnRDaG9vc2VyKGlkLCBrZXksIHBhcmVudHMpO1xyXG4gICAgICAgIHZhbHVlcy5wdXNoKHBhcmVudHNbcF1ba2V5XVtpXSk7XHJcbiAgICAgIH1cclxuICAgICAgY3Jvc3NEZWZba2V5XSA9IHZhbHVlcztcclxuICAgICAgcmV0dXJuIGNyb3NzRGVmO1xyXG4gICAgfSwge1xyXG4gICAgICBpZDogaWQsXHJcbiAgICAgIGFuY2VzdHJ5OiBwYXJlbnRzLm1hcChmdW5jdGlvbihwYXJlbnQpe1xyXG4gICAgICAgIHJldHVybiB7XHJcbiAgICAgICAgICBpZDogcGFyZW50LmlkLFxyXG4gICAgICAgICAgYW5jZXN0cnk6IHBhcmVudC5hbmNlc3RyeSxcclxuICAgICAgICB9O1xyXG4gICAgICB9KVxyXG4gICAgfSk7XHJcbiAgfSxcclxuICBjcmVhdGVNdXRhdGVkQ2xvbmUoc2NoZW1hLCBnZW5lcmF0b3IsIHBhcmVudCwgZmFjdG9yLCBjaGFuY2VUb011dGF0ZSl7XHJcbiAgICByZXR1cm4gT2JqZWN0LmtleXMoc2NoZW1hKS5yZWR1Y2UoZnVuY3Rpb24oY2xvbmUsIGtleSl7XHJcbiAgICAgIHZhciBzY2hlbWFQcm9wID0gc2NoZW1hW2tleV07XHJcbiAgICAgIHZhciBvcmlnaW5hbFZhbHVlcyA9IHBhcmVudFtrZXldO1xyXG4gICAgICB2YXIgdmFsdWVzID0gcmFuZG9tLm11dGF0ZU5vcm1hbHMoXHJcbiAgICAgICAgc2NoZW1hUHJvcCwgZ2VuZXJhdG9yLCBvcmlnaW5hbFZhbHVlcywgZmFjdG9yLCBjaGFuY2VUb011dGF0ZVxyXG4gICAgICApO1xyXG4gICAgICBjbG9uZVtrZXldID0gdmFsdWVzO1xyXG4gICAgICByZXR1cm4gY2xvbmU7XHJcbiAgICB9LCB7XHJcbiAgICAgIGlkOiBwYXJlbnQuaWQsXHJcbiAgICAgIGFuY2VzdHJ5OiBwYXJlbnQuYW5jZXN0cnlcclxuICAgIH0pO1xyXG4gIH0sXHJcbiAgYXBwbHlUeXBlcyhzY2hlbWEsIHBhcmVudCl7XHJcbiAgICByZXR1cm4gT2JqZWN0LmtleXMoc2NoZW1hKS5yZWR1Y2UoZnVuY3Rpb24oY2xvbmUsIGtleSl7XHJcbiAgICAgIHZhciBzY2hlbWFQcm9wID0gc2NoZW1hW2tleV07XHJcbiAgICAgIHZhciBvcmlnaW5hbFZhbHVlcyA9IHBhcmVudFtrZXldO1xyXG4gICAgICB2YXIgdmFsdWVzO1xyXG4gICAgICBzd2l0Y2goc2NoZW1hUHJvcC50eXBlKXtcclxuICAgICAgICBjYXNlIFwic2h1ZmZsZVwiIDpcclxuICAgICAgICAgIHZhbHVlcyA9IHJhbmRvbS5tYXBUb1NodWZmbGUoc2NoZW1hUHJvcCwgb3JpZ2luYWxWYWx1ZXMpOyBicmVhaztcclxuICAgICAgICBjYXNlIFwiZmxvYXRcIiA6XHJcbiAgICAgICAgICB2YWx1ZXMgPSByYW5kb20ubWFwVG9GbG9hdChzY2hlbWFQcm9wLCBvcmlnaW5hbFZhbHVlcyk7IGJyZWFrO1xyXG4gICAgICAgIGNhc2UgXCJpbnRlZ2VyXCI6XHJcbiAgICAgICAgICB2YWx1ZXMgPSByYW5kb20ubWFwVG9JbnRlZ2VyKHNjaGVtYVByb3AsIG9yaWdpbmFsVmFsdWVzKTsgYnJlYWs7XHJcbiAgICAgICAgZGVmYXVsdDpcclxuICAgICAgICAgIHRocm93IG5ldyBFcnJvcihgVW5rbm93biB0eXBlICR7c2NoZW1hUHJvcC50eXBlfSBvZiBzY2hlbWEgZm9yIGtleSAke2tleX1gKTtcclxuICAgICAgfVxyXG4gICAgICBjbG9uZVtrZXldID0gdmFsdWVzO1xyXG4gICAgICByZXR1cm4gY2xvbmU7XHJcbiAgICB9LCB7XHJcbiAgICAgIGlkOiBwYXJlbnQuaWQsXHJcbiAgICAgIGFuY2VzdHJ5OiBwYXJlbnQuYW5jZXN0cnlcclxuICAgIH0pO1xyXG4gIH0sXHJcbn1cclxuIiwidmFyIGNyZWF0ZSA9IHJlcXVpcmUoXCIuLi9jcmVhdGUtaW5zdGFuY2VcIik7XHJcblxyXG5tb2R1bGUuZXhwb3J0cyA9IHtcclxuICBnZW5lcmF0aW9uWmVybzogZ2VuZXJhdGlvblplcm8sXHJcbiAgbmV4dEdlbmVyYXRpb246IG5leHRHZW5lcmF0aW9uXHJcbn1cclxuXHJcbmZ1bmN0aW9uIGdlbmVyYXRpb25aZXJvKGNvbmZpZyl7XHJcbiAgdmFyIGdlbmVyYXRpb25TaXplID0gY29uZmlnLmdlbmVyYXRpb25TaXplLFxyXG4gIHNjaGVtYSA9IGNvbmZpZy5zY2hlbWE7XHJcbiAgdmFyIGN3X2NhckdlbmVyYXRpb24gPSBbXTtcclxuICBmb3IgKHZhciBrID0gMDsgayA8IGdlbmVyYXRpb25TaXplOyBrKyspIHtcclxuICAgIHZhciBkZWYgPSBjcmVhdGUuY3JlYXRlR2VuZXJhdGlvblplcm8oc2NoZW1hLCBmdW5jdGlvbigpe1xyXG4gICAgICByZXR1cm4gTWF0aC5yYW5kb20oKVxyXG4gICAgfSk7XHJcbiAgICBkZWYuaW5kZXggPSBrO1xyXG4gICAgY3dfY2FyR2VuZXJhdGlvbi5wdXNoKGRlZik7XHJcbiAgfVxyXG4gIHJldHVybiB7XHJcbiAgICBjb3VudGVyOiAwLFxyXG4gICAgZ2VuZXJhdGlvbjogY3dfY2FyR2VuZXJhdGlvbixcclxuICB9O1xyXG59XHJcblxyXG5mdW5jdGlvbiBuZXh0R2VuZXJhdGlvbihcclxuICBwcmV2aW91c1N0YXRlLFxyXG4gIHNjb3JlcyxcclxuICBjb25maWdcclxuKXtcclxuICB2YXIgY2hhbXBpb25fbGVuZ3RoID0gY29uZmlnLmNoYW1waW9uTGVuZ3RoLFxyXG4gICAgZ2VuZXJhdGlvblNpemUgPSBjb25maWcuZ2VuZXJhdGlvblNpemUsXHJcbiAgICBzZWxlY3RGcm9tQWxsUGFyZW50cyA9IGNvbmZpZy5zZWxlY3RGcm9tQWxsUGFyZW50cztcclxuXHJcbiAgdmFyIG5ld0dlbmVyYXRpb24gPSBuZXcgQXJyYXkoKTtcclxuICB2YXIgbmV3Ym9ybjtcclxuICBmb3IgKHZhciBrID0gMDsgayA8IGNoYW1waW9uX2xlbmd0aDsgaysrKSB7YGBcclxuICAgIHNjb3Jlc1trXS5kZWYuaXNfZWxpdGUgPSB0cnVlO1xyXG4gICAgc2NvcmVzW2tdLmRlZi5pbmRleCA9IGs7XHJcbiAgICBuZXdHZW5lcmF0aW9uLnB1c2goc2NvcmVzW2tdLmRlZik7XHJcbiAgfVxyXG4gIHZhciBwYXJlbnRMaXN0ID0gW107XHJcbiAgZm9yIChrID0gY2hhbXBpb25fbGVuZ3RoOyBrIDwgZ2VuZXJhdGlvblNpemU7IGsrKykge1xyXG4gICAgdmFyIHBhcmVudDEgPSBzZWxlY3RGcm9tQWxsUGFyZW50cyhzY29yZXMsIHBhcmVudExpc3QpO1xyXG4gICAgdmFyIHBhcmVudDIgPSBwYXJlbnQxO1xyXG4gICAgd2hpbGUgKHBhcmVudDIgPT0gcGFyZW50MSkge1xyXG4gICAgICBwYXJlbnQyID0gc2VsZWN0RnJvbUFsbFBhcmVudHMoc2NvcmVzLCBwYXJlbnRMaXN0LCBwYXJlbnQxKTtcclxuICAgIH1cclxuICAgIHZhciBwYWlyID0gW3BhcmVudDEsIHBhcmVudDJdXHJcbiAgICBwYXJlbnRMaXN0LnB1c2gocGFpcik7XHJcbiAgICBuZXdib3JuID0gbWFrZUNoaWxkKGNvbmZpZyxcclxuICAgICAgcGFpci5tYXAoZnVuY3Rpb24ocGFyZW50KSB7IHJldHVybiBzY29yZXNbcGFyZW50XS5kZWY7IH0pXHJcbiAgICApO1xyXG4gICAgbmV3Ym9ybiA9IG11dGF0ZShjb25maWcsIG5ld2Jvcm4pO1xyXG4gICAgbmV3Ym9ybi5pc19lbGl0ZSA9IGZhbHNlO1xyXG4gICAgbmV3Ym9ybi5pbmRleCA9IGs7XHJcbiAgICBuZXdHZW5lcmF0aW9uLnB1c2gobmV3Ym9ybik7XHJcbiAgfVxyXG5cclxuICByZXR1cm4ge1xyXG4gICAgY291bnRlcjogcHJldmlvdXNTdGF0ZS5jb3VudGVyICsgMSxcclxuICAgIGdlbmVyYXRpb246IG5ld0dlbmVyYXRpb24sXHJcbiAgfTtcclxufVxyXG5cclxuXHJcbmZ1bmN0aW9uIG1ha2VDaGlsZChjb25maWcsIHBhcmVudHMpe1xyXG4gIHZhciBzY2hlbWEgPSBjb25maWcuc2NoZW1hLFxyXG4gICAgcGlja1BhcmVudCA9IGNvbmZpZy5waWNrUGFyZW50O1xyXG4gIHJldHVybiBjcmVhdGUuY3JlYXRlQ3Jvc3NCcmVlZChzY2hlbWEsIHBhcmVudHMsIHBpY2tQYXJlbnQpXHJcbn1cclxuXHJcblxyXG5mdW5jdGlvbiBtdXRhdGUoY29uZmlnLCBwYXJlbnQpe1xyXG4gIHZhciBzY2hlbWEgPSBjb25maWcuc2NoZW1hLFxyXG4gICAgbXV0YXRpb25fcmFuZ2UgPSBjb25maWcubXV0YXRpb25fcmFuZ2UsXHJcbiAgICBnZW5fbXV0YXRpb24gPSBjb25maWcuZ2VuX211dGF0aW9uLFxyXG4gICAgZ2VuZXJhdGVSYW5kb20gPSBjb25maWcuZ2VuZXJhdGVSYW5kb207XHJcbiAgcmV0dXJuIGNyZWF0ZS5jcmVhdGVNdXRhdGVkQ2xvbmUoXHJcbiAgICBzY2hlbWEsXHJcbiAgICBnZW5lcmF0ZVJhbmRvbSxcclxuICAgIHBhcmVudCxcclxuICAgIE1hdGgubWF4KG11dGF0aW9uX3JhbmdlKSxcclxuICAgIGdlbl9tdXRhdGlvblxyXG4gIClcclxufVxyXG4iLCJcclxuXHJcbmNvbnN0IHJhbmRvbSA9IHtcclxuICBzaHVmZmxlSW50ZWdlcnMocHJvcCwgZ2VuZXJhdG9yKXtcclxuICAgIHJldHVybiByYW5kb20ubWFwVG9TaHVmZmxlKHByb3AsIHJhbmRvbS5jcmVhdGVOb3JtYWxzKHtcclxuICAgICAgbGVuZ3RoOiBwcm9wLmxlbmd0aCB8fCAxMCxcclxuICAgICAgaW5jbHVzaXZlOiB0cnVlLFxyXG4gICAgfSwgZ2VuZXJhdG9yKSk7XHJcbiAgfSxcclxuICBjcmVhdGVJbnRlZ2Vycyhwcm9wLCBnZW5lcmF0b3Ipe1xyXG4gICAgcmV0dXJuIHJhbmRvbS5tYXBUb0ludGVnZXIocHJvcCwgcmFuZG9tLmNyZWF0ZU5vcm1hbHMoe1xyXG4gICAgICBsZW5ndGg6IHByb3AubGVuZ3RoLFxyXG4gICAgICBpbmNsdXNpdmU6IHRydWUsXHJcbiAgICB9LCBnZW5lcmF0b3IpKTtcclxuICB9LFxyXG4gIGNyZWF0ZUZsb2F0cyhwcm9wLCBnZW5lcmF0b3Ipe1xyXG4gICAgcmV0dXJuIHJhbmRvbS5tYXBUb0Zsb2F0KHByb3AsIHJhbmRvbS5jcmVhdGVOb3JtYWxzKHtcclxuICAgICAgbGVuZ3RoOiBwcm9wLmxlbmd0aCxcclxuICAgICAgaW5jbHVzaXZlOiB0cnVlLFxyXG4gICAgfSwgZ2VuZXJhdG9yKSk7XHJcbiAgfSxcclxuICBjcmVhdGVOb3JtYWxzKHByb3AsIGdlbmVyYXRvcil7XHJcbiAgICB2YXIgbCA9IHByb3AubGVuZ3RoO1xyXG4gICAgdmFyIHZhbHVlcyA9IFtdO1xyXG4gICAgZm9yKHZhciBpID0gMDsgaSA8IGw7IGkrKyl7XHJcbiAgICAgIHZhbHVlcy5wdXNoKFxyXG4gICAgICAgIGNyZWF0ZU5vcm1hbChwcm9wLCBnZW5lcmF0b3IpXHJcbiAgICAgICk7XHJcbiAgICB9XHJcbiAgICByZXR1cm4gdmFsdWVzO1xyXG4gIH0sXHJcbiAgbXV0YXRlU2h1ZmZsZShcclxuICAgIHByb3AsIGdlbmVyYXRvciwgb3JpZ2luYWxWYWx1ZXMsIG11dGF0aW9uX3JhbmdlLCBjaGFuY2VUb011dGF0ZVxyXG4gICl7XHJcbiAgICByZXR1cm4gcmFuZG9tLm1hcFRvU2h1ZmZsZShwcm9wLCByYW5kb20ubXV0YXRlTm9ybWFscyhcclxuICAgICAgcHJvcCwgZ2VuZXJhdG9yLCBvcmlnaW5hbFZhbHVlcywgbXV0YXRpb25fcmFuZ2UsIGNoYW5jZVRvTXV0YXRlXHJcbiAgICApKTtcclxuICB9LFxyXG4gIG11dGF0ZUludGVnZXJzKHByb3AsIGdlbmVyYXRvciwgb3JpZ2luYWxWYWx1ZXMsIG11dGF0aW9uX3JhbmdlLCBjaGFuY2VUb011dGF0ZSl7XHJcbiAgICByZXR1cm4gcmFuZG9tLm1hcFRvSW50ZWdlcihwcm9wLCByYW5kb20ubXV0YXRlTm9ybWFscyhcclxuICAgICAgcHJvcCwgZ2VuZXJhdG9yLCBvcmlnaW5hbFZhbHVlcywgbXV0YXRpb25fcmFuZ2UsIGNoYW5jZVRvTXV0YXRlXHJcbiAgICApKTtcclxuICB9LFxyXG4gIG11dGF0ZUZsb2F0cyhwcm9wLCBnZW5lcmF0b3IsIG9yaWdpbmFsVmFsdWVzLCBtdXRhdGlvbl9yYW5nZSwgY2hhbmNlVG9NdXRhdGUpe1xyXG4gICAgcmV0dXJuIHJhbmRvbS5tYXBUb0Zsb2F0KHByb3AsIHJhbmRvbS5tdXRhdGVOb3JtYWxzKFxyXG4gICAgICBwcm9wLCBnZW5lcmF0b3IsIG9yaWdpbmFsVmFsdWVzLCBtdXRhdGlvbl9yYW5nZSwgY2hhbmNlVG9NdXRhdGVcclxuICAgICkpO1xyXG4gIH0sXHJcbiAgbWFwVG9TaHVmZmxlKHByb3AsIG5vcm1hbHMpe1xyXG4gICAgdmFyIG9mZnNldCA9IHByb3Aub2Zmc2V0IHx8IDA7XHJcbiAgICB2YXIgbGltaXQgPSBwcm9wLmxpbWl0IHx8IHByb3AubGVuZ3RoO1xyXG4gICAgdmFyIHNvcnRlZCA9IG5vcm1hbHMuc2xpY2UoKS5zb3J0KGZ1bmN0aW9uKGEsIGIpe1xyXG4gICAgICByZXR1cm4gYSAtIGI7XHJcbiAgICB9KTtcclxuICAgIHJldHVybiBub3JtYWxzLm1hcChmdW5jdGlvbih2YWwpe1xyXG4gICAgICByZXR1cm4gc29ydGVkLmluZGV4T2YodmFsKTtcclxuICAgIH0pLm1hcChmdW5jdGlvbihpKXtcclxuICAgICAgcmV0dXJuIGkgKyBvZmZzZXQ7XHJcbiAgICB9KS5zbGljZSgwLCBsaW1pdCk7XHJcbiAgfSxcclxuICBtYXBUb0ludGVnZXIocHJvcCwgbm9ybWFscyl7XHJcbiAgICBwcm9wID0ge1xyXG4gICAgICBtaW46IHByb3AubWluIHx8IDAsXHJcbiAgICAgIHJhbmdlOiBwcm9wLnJhbmdlIHx8IDEwLFxyXG4gICAgICBsZW5ndGg6IHByb3AubGVuZ3RoXHJcbiAgICB9XHJcbiAgICByZXR1cm4gcmFuZG9tLm1hcFRvRmxvYXQocHJvcCwgbm9ybWFscykubWFwKGZ1bmN0aW9uKGZsb2F0KXtcclxuICAgICAgcmV0dXJuIE1hdGgucm91bmQoZmxvYXQpO1xyXG4gICAgfSk7XHJcbiAgfSxcclxuICBtYXBUb0Zsb2F0KHByb3AsIG5vcm1hbHMpe1xyXG4gICAgcHJvcCA9IHtcclxuICAgICAgbWluOiBwcm9wLm1pbiB8fCAwLFxyXG4gICAgICByYW5nZTogcHJvcC5yYW5nZSB8fCAxXHJcbiAgICB9XHJcbiAgICByZXR1cm4gbm9ybWFscy5tYXAoZnVuY3Rpb24obm9ybWFsKXtcclxuICAgICAgdmFyIG1pbiA9IHByb3AubWluO1xyXG4gICAgICB2YXIgcmFuZ2UgPSBwcm9wLnJhbmdlO1xyXG4gICAgICByZXR1cm4gbWluICsgbm9ybWFsICogcmFuZ2VcclxuICAgIH0pXHJcbiAgfSxcclxuICBtdXRhdGVOb3JtYWxzKHByb3AsIGdlbmVyYXRvciwgb3JpZ2luYWxWYWx1ZXMsIG11dGF0aW9uX3JhbmdlLCBjaGFuY2VUb011dGF0ZSl7XHJcbiAgICB2YXIgZmFjdG9yID0gKHByb3AuZmFjdG9yIHx8IDEpICogbXV0YXRpb25fcmFuZ2VcclxuICAgIHJldHVybiBvcmlnaW5hbFZhbHVlcy5tYXAoZnVuY3Rpb24ob3JpZ2luYWxWYWx1ZSl7XHJcbiAgICAgIGlmKGdlbmVyYXRvcigpID4gY2hhbmNlVG9NdXRhdGUpe1xyXG4gICAgICAgIHJldHVybiBvcmlnaW5hbFZhbHVlO1xyXG4gICAgICB9XHJcbiAgICAgIHJldHVybiBtdXRhdGVOb3JtYWwoXHJcbiAgICAgICAgcHJvcCwgZ2VuZXJhdG9yLCBvcmlnaW5hbFZhbHVlLCBmYWN0b3JcclxuICAgICAgKTtcclxuICAgIH0pO1xyXG4gIH1cclxufTtcclxuXHJcbm1vZHVsZS5leHBvcnRzID0gcmFuZG9tO1xyXG5cclxuZnVuY3Rpb24gbXV0YXRlTm9ybWFsKHByb3AsIGdlbmVyYXRvciwgb3JpZ2luYWxWYWx1ZSwgbXV0YXRpb25fcmFuZ2Upe1xyXG4gIGlmKG11dGF0aW9uX3JhbmdlID4gMSl7XHJcbiAgICB0aHJvdyBuZXcgRXJyb3IoXCJDYW5ub3QgbXV0YXRlIGJleW9uZCBib3VuZHNcIik7XHJcbiAgfVxyXG4gIHZhciBuZXdNaW4gPSBvcmlnaW5hbFZhbHVlIC0gMC41O1xyXG4gIGlmIChuZXdNaW4gPCAwKSBuZXdNaW4gPSAwO1xyXG4gIGlmIChuZXdNaW4gKyBtdXRhdGlvbl9yYW5nZSAgPiAxKVxyXG4gICAgbmV3TWluID0gMSAtIG11dGF0aW9uX3JhbmdlO1xyXG4gIHZhciByYW5nZVZhbHVlID0gY3JlYXRlTm9ybWFsKHtcclxuICAgIGluY2x1c2l2ZTogdHJ1ZSxcclxuICB9LCBnZW5lcmF0b3IpO1xyXG4gIHJldHVybiBuZXdNaW4gKyByYW5nZVZhbHVlICogbXV0YXRpb25fcmFuZ2U7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGNyZWF0ZU5vcm1hbChwcm9wLCBnZW5lcmF0b3Ipe1xyXG4gIGlmKCFwcm9wLmluY2x1c2l2ZSl7XHJcbiAgICByZXR1cm4gZ2VuZXJhdG9yKCk7XHJcbiAgfSBlbHNlIHtcclxuICAgIHJldHVybiBnZW5lcmF0b3IoKSA8IDAuNSA/XHJcbiAgICBnZW5lcmF0b3IoKSA6XHJcbiAgICAxIC0gZ2VuZXJhdG9yKCk7XHJcbiAgfVxyXG59XHJcbiIsIi8qIGdsb2JhbHMgYnRvYSAqL1xyXG52YXIgc2V0dXBTY2VuZSA9IHJlcXVpcmUoXCIuL3NldHVwLXNjZW5lXCIpO1xyXG52YXIgY2FyUnVuID0gcmVxdWlyZShcIi4uL2Nhci1zY2hlbWEvcnVuXCIpO1xyXG52YXIgZGVmVG9DYXIgPSByZXF1aXJlKFwiLi4vY2FyLXNjaGVtYS9kZWYtdG8tY2FyXCIpO1xyXG5cclxubW9kdWxlLmV4cG9ydHMgPSBydW5EZWZzO1xyXG5mdW5jdGlvbiBydW5EZWZzKHdvcmxkX2RlZiwgZGVmcywgbGlzdGVuZXJzKSB7XHJcbiAgaWYgKHdvcmxkX2RlZi5tdXRhYmxlX2Zsb29yKSB7XHJcbiAgICAvLyBHSE9TVCBESVNBQkxFRFxyXG4gICAgd29ybGRfZGVmLmZsb29yc2VlZCA9IGJ0b2EoTWF0aC5zZWVkcmFuZG9tKCkpO1xyXG4gIH1cclxuXHJcbiAgdmFyIHNjZW5lID0gc2V0dXBTY2VuZSh3b3JsZF9kZWYpO1xyXG4gIHNjZW5lLndvcmxkLlN0ZXAoMSAvIHdvcmxkX2RlZi5ib3gyZGZwcywgMjAsIDIwKTtcclxuICBjb25zb2xlLmxvZyhcImFib3V0IHRvIGJ1aWxkIGNhcnNcIik7XHJcbiAgdmFyIGNhcnMgPSBkZWZzLm1hcCgoZGVmLCBpKSA9PiB7XHJcbiAgICByZXR1cm4ge1xyXG4gICAgICBpbmRleDogaSxcclxuICAgICAgZGVmOiBkZWYsXHJcbiAgICAgIGNhcjogZGVmVG9DYXIoZGVmLCBzY2VuZS53b3JsZCwgd29ybGRfZGVmKSxcclxuICAgICAgc3RhdGU6IGNhclJ1bi5nZXRJbml0aWFsU3RhdGUod29ybGRfZGVmKVxyXG4gICAgfTtcclxuICB9KTtcclxuICB2YXIgYWxpdmVjYXJzID0gY2FycztcclxuICByZXR1cm4ge1xyXG4gICAgc2NlbmU6IHNjZW5lLFxyXG4gICAgY2FyczogY2FycyxcclxuICAgIHN0ZXA6IGZ1bmN0aW9uICgpIHtcclxuICAgICAgaWYgKGFsaXZlY2Fycy5sZW5ndGggPT09IDApIHtcclxuICAgICAgICB0aHJvdyBuZXcgRXJyb3IoXCJubyBtb3JlIGNhcnNcIik7XHJcbiAgICAgIH1cclxuICAgICAgc2NlbmUud29ybGQuU3RlcCgxIC8gd29ybGRfZGVmLmJveDJkZnBzLCAyMCwgMjApO1xyXG4gICAgICBsaXN0ZW5lcnMucHJlQ2FyU3RlcCgpO1xyXG4gICAgICBhbGl2ZWNhcnMgPSBhbGl2ZWNhcnMuZmlsdGVyKGZ1bmN0aW9uIChjYXIpIHtcclxuICAgICAgICBjYXIuc3RhdGUgPSBjYXJSdW4udXBkYXRlU3RhdGUoXHJcbiAgICAgICAgICB3b3JsZF9kZWYsIGNhci5jYXIsIGNhci5zdGF0ZVxyXG4gICAgICAgICk7XHJcbiAgICAgICAgdmFyIHN0YXR1cyA9IGNhclJ1bi5nZXRTdGF0dXMoY2FyLnN0YXRlLCB3b3JsZF9kZWYpO1xyXG4gICAgICAgIGxpc3RlbmVycy5jYXJTdGVwKGNhcik7XHJcbiAgICAgICAgaWYgKHN0YXR1cyA9PT0gMCkge1xyXG4gICAgICAgICAgcmV0dXJuIHRydWU7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGNhci5zY29yZSA9IGNhclJ1bi5jYWxjdWxhdGVTY29yZShjYXIuc3RhdGUsIHdvcmxkX2RlZik7XHJcbiAgICAgICAgbGlzdGVuZXJzLmNhckRlYXRoKGNhcik7XHJcblxyXG4gICAgICAgIHZhciB3b3JsZCA9IHNjZW5lLndvcmxkO1xyXG4gICAgICAgIHZhciB3b3JsZENhciA9IGNhci5jYXI7XHJcbiAgICAgICAgd29ybGQuRGVzdHJveUJvZHkod29ybGRDYXIuY2hhc3Npcyk7XHJcblxyXG4gICAgICAgIGZvciAodmFyIHcgPSAwOyB3IDwgd29ybGRDYXIud2hlZWxzLmxlbmd0aDsgdysrKSB7XHJcbiAgICAgICAgICB3b3JsZC5EZXN0cm95Qm9keSh3b3JsZENhci53aGVlbHNbd10pO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgICB9KVxyXG4gICAgICBpZiAoYWxpdmVjYXJzLmxlbmd0aCA9PT0gMCkge1xyXG4gICAgICAgIGxpc3RlbmVycy5nZW5lcmF0aW9uRW5kKGNhcnMpO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG5cclxufVxyXG4iLCIvKiBnbG9iYWxzIGIyV29ybGQgYjJWZWMyIGIyQm9keURlZiBiMkZpeHR1cmVEZWYgYjJQb2x5Z29uU2hhcGUgKi9cclxuXHJcbi8qXHJcblxyXG53b3JsZF9kZWYgPSB7XHJcbiAgZ3Jhdml0eToge3gsIHl9LFxyXG4gIGRvU2xlZXA6IGJvb2xlYW4sXHJcbiAgZmxvb3JzZWVkOiBzdHJpbmcsXHJcbiAgdGlsZURpbWVuc2lvbnMsXHJcbiAgbWF4Rmxvb3JUaWxlcyxcclxuICBtdXRhYmxlX2Zsb29yOiBib29sZWFuXHJcbn1cclxuXHJcbiovXHJcblxyXG5tb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uKHdvcmxkX2RlZil7XHJcblxyXG4gIHZhciB3b3JsZCA9IG5ldyBiMldvcmxkKHdvcmxkX2RlZi5ncmF2aXR5LCB3b3JsZF9kZWYuZG9TbGVlcCk7XHJcbiAgdmFyIGZsb29yVGlsZXMgPSBjd19jcmVhdGVGbG9vcihcclxuICAgIHdvcmxkLFxyXG4gICAgd29ybGRfZGVmLmZsb29yc2VlZCxcclxuICAgIHdvcmxkX2RlZi50aWxlRGltZW5zaW9ucyxcclxuICAgIHdvcmxkX2RlZi5tYXhGbG9vclRpbGVzLFxyXG4gICAgd29ybGRfZGVmLm11dGFibGVfZmxvb3JcclxuICApO1xyXG5cclxuICB2YXIgbGFzdF90aWxlID0gZmxvb3JUaWxlc1tcclxuICAgIGZsb29yVGlsZXMubGVuZ3RoIC0gMVxyXG4gIF07XHJcbiAgdmFyIGxhc3RfZml4dHVyZSA9IGxhc3RfdGlsZS5HZXRGaXh0dXJlTGlzdCgpO1xyXG4gIHZhciB0aWxlX3Bvc2l0aW9uID0gbGFzdF90aWxlLkdldFdvcmxkUG9pbnQoXHJcbiAgICBsYXN0X2ZpeHR1cmUuR2V0U2hhcGUoKS5tX3ZlcnRpY2VzWzNdXHJcbiAgKTtcclxuICB3b3JsZC5maW5pc2hMaW5lID0gdGlsZV9wb3NpdGlvbi54O1xyXG4gIHJldHVybiB7XHJcbiAgICB3b3JsZDogd29ybGQsXHJcbiAgICBmbG9vclRpbGVzOiBmbG9vclRpbGVzLFxyXG4gICAgZmluaXNoTGluZTogdGlsZV9wb3NpdGlvbi54XHJcbiAgfTtcclxufVxyXG5cclxuZnVuY3Rpb24gY3dfY3JlYXRlRmxvb3Iod29ybGQsIGZsb29yc2VlZCwgZGltZW5zaW9ucywgbWF4Rmxvb3JUaWxlcywgbXV0YWJsZV9mbG9vcikge1xyXG4gIHZhciBsYXN0X3RpbGUgPSBudWxsO1xyXG4gIHZhciB0aWxlX3Bvc2l0aW9uID0gbmV3IGIyVmVjMigtNSwgMCk7XHJcbiAgdmFyIGN3X2Zsb29yVGlsZXMgPSBbXTtcclxuICBNYXRoLnNlZWRyYW5kb20oZmxvb3JzZWVkKTtcclxuICBmb3IgKHZhciBrID0gMDsgayA8IG1heEZsb29yVGlsZXM7IGsrKykge1xyXG4gICAgaWYgKCFtdXRhYmxlX2Zsb29yKSB7XHJcbiAgICAgIC8vIGtlZXAgb2xkIGltcG9zc2libGUgdHJhY2tzIGlmIG5vdCB1c2luZyBtdXRhYmxlIGZsb29yc1xyXG4gICAgICBsYXN0X3RpbGUgPSBjd19jcmVhdGVGbG9vclRpbGUoXHJcbiAgICAgICAgd29ybGQsIGRpbWVuc2lvbnMsIHRpbGVfcG9zaXRpb24sIChNYXRoLnJhbmRvbSgpICogMyAtIDEuNSkgKiAxLjUgKiBrIC8gbWF4Rmxvb3JUaWxlc1xyXG4gICAgICApO1xyXG4gICAgfSBlbHNlIHtcclxuICAgICAgLy8gaWYgcGF0aCBpcyBtdXRhYmxlIG92ZXIgcmFjZXMsIGNyZWF0ZSBzbW9vdGhlciB0cmFja3NcclxuICAgICAgbGFzdF90aWxlID0gY3dfY3JlYXRlRmxvb3JUaWxlKFxyXG4gICAgICAgIHdvcmxkLCBkaW1lbnNpb25zLCB0aWxlX3Bvc2l0aW9uLCAoTWF0aC5yYW5kb20oKSAqIDMgLSAxLjUpICogMS4yICogayAvIG1heEZsb29yVGlsZXNcclxuICAgICAgKTtcclxuICAgIH1cclxuICAgIGN3X2Zsb29yVGlsZXMucHVzaChsYXN0X3RpbGUpO1xyXG4gICAgdmFyIGxhc3RfZml4dHVyZSA9IGxhc3RfdGlsZS5HZXRGaXh0dXJlTGlzdCgpO1xyXG4gICAgdGlsZV9wb3NpdGlvbiA9IGxhc3RfdGlsZS5HZXRXb3JsZFBvaW50KGxhc3RfZml4dHVyZS5HZXRTaGFwZSgpLm1fdmVydGljZXNbM10pO1xyXG4gIH1cclxuICByZXR1cm4gY3dfZmxvb3JUaWxlcztcclxufVxyXG5cclxuXHJcbmZ1bmN0aW9uIGN3X2NyZWF0ZUZsb29yVGlsZSh3b3JsZCwgZGltLCBwb3NpdGlvbiwgYW5nbGUpIHtcclxuICB2YXIgYm9keV9kZWYgPSBuZXcgYjJCb2R5RGVmKCk7XHJcblxyXG4gIGJvZHlfZGVmLnBvc2l0aW9uLlNldChwb3NpdGlvbi54LCBwb3NpdGlvbi55KTtcclxuICB2YXIgYm9keSA9IHdvcmxkLkNyZWF0ZUJvZHkoYm9keV9kZWYpO1xyXG4gIHZhciBmaXhfZGVmID0gbmV3IGIyRml4dHVyZURlZigpO1xyXG4gIGZpeF9kZWYuc2hhcGUgPSBuZXcgYjJQb2x5Z29uU2hhcGUoKTtcclxuICBmaXhfZGVmLmZyaWN0aW9uID0gMC41O1xyXG5cclxuICB2YXIgY29vcmRzID0gbmV3IEFycmF5KCk7XHJcbiAgY29vcmRzLnB1c2gobmV3IGIyVmVjMigwLCAwKSk7XHJcbiAgY29vcmRzLnB1c2gobmV3IGIyVmVjMigwLCAtZGltLnkpKTtcclxuICBjb29yZHMucHVzaChuZXcgYjJWZWMyKGRpbS54LCAtZGltLnkpKTtcclxuICBjb29yZHMucHVzaChuZXcgYjJWZWMyKGRpbS54LCAwKSk7XHJcblxyXG4gIHZhciBjZW50ZXIgPSBuZXcgYjJWZWMyKDAsIDApO1xyXG5cclxuICB2YXIgbmV3Y29vcmRzID0gY3dfcm90YXRlRmxvb3JUaWxlKGNvb3JkcywgY2VudGVyLCBhbmdsZSk7XHJcblxyXG4gIGZpeF9kZWYuc2hhcGUuU2V0QXNBcnJheShuZXdjb29yZHMpO1xyXG5cclxuICBib2R5LkNyZWF0ZUZpeHR1cmUoZml4X2RlZik7XHJcbiAgcmV0dXJuIGJvZHk7XHJcbn1cclxuXHJcbmZ1bmN0aW9uIGN3X3JvdGF0ZUZsb29yVGlsZShjb29yZHMsIGNlbnRlciwgYW5nbGUpIHtcclxuICByZXR1cm4gY29vcmRzLm1hcChmdW5jdGlvbihjb29yZCl7XHJcbiAgICByZXR1cm4ge1xyXG4gICAgICB4OiBNYXRoLmNvcyhhbmdsZSkgKiAoY29vcmQueCAtIGNlbnRlci54KSAtIE1hdGguc2luKGFuZ2xlKSAqIChjb29yZC55IC0gY2VudGVyLnkpICsgY2VudGVyLngsXHJcbiAgICAgIHk6IE1hdGguc2luKGFuZ2xlKSAqIChjb29yZC54IC0gY2VudGVyLngpICsgTWF0aC5jb3MoYW5nbGUpICogKGNvb3JkLnkgLSBjZW50ZXIueSkgKyBjZW50ZXIueSxcclxuICAgIH07XHJcbiAgfSk7XHJcbn1cclxuIl19
