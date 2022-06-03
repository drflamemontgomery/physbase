package physbase.physics;

import maths.Types;

class PhysicsManager {

    public static var gravity:Vec2;
    private var bodies:Array<Body>;

    public static var cell_size:Int;
    
    public function new(g:Vec2, cs:Int) {

        gravity = g;
        cell_size = cs;
	
	bodies = new Array<Body>();
    }

    public function update(steps:Int, dt:Float) {
	var timeStep = dt/steps;

	for(step in 0...steps) {
	    for(body in bodies) {
		body.update(timeStep, gravity);
	    }
	    for(body in bodies) {
		for(bd in bodies) {
		    body.collide(bd);
		}
	    }

	    for(body in bodies) {
		body.fixNewCollisions();
	    }
	}
    }

    public function createBody():Body {
	var body = new Body();
	bodies.push(body);
	return body;
    }
}
