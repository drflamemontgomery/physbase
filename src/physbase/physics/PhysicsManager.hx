package physbase.physics;

import physbase.maths.Types;

class PhysicsManager {

    public static var gravity:Vec2;
    private final bodies:Array<Body>;

    public static var cell_size:Int;
    public static var half_pixel:Float;
    
    public function new(g:Vec2, cs:Int) {
        gravity = g;
        cell_size = cs;
	half_pixel = 0.5/cs;
	
	bodies = new Array<Body>();
    }

    public function update(steps:Int, dt:Float) {
	final timeStep = dt/steps;

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
		body.fixPosition();
	    }
	}
    }

    public function createBody(?constructor:Void->Body):Body {
	if(constructor == null) {
	    constructor = Body.new;
	}
        final body:Body = constructor();
	bodies.push(body);
	return body;
    }
}
