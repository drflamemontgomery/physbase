package physbase.physics;

import physbase.maths.Types;

class Body {
    public var cx : Int;
    public var cy : Int;
    public var xr : Float;
    public var yr : Float;

    public var isStatic : Bool;
    public var isSensor : Bool;
    
    public var xx(default, null) : Float;
    public var yy(default, null) : Float;

    public var dx : Float;
    public var dy : Float;
    public var gravityScale : Float;
    public var restitution : Float;

    public var radius : Float;

    public var new_collisions : Array<Body>;
    public var collisions : Array<Body>;
    public var old_collisions : Array<Body>;

    public var enteredEvents : Array<Body->Void>;
    public var exitedEvents : Array<Body->Void>;

    public function attachOnEntered(event:Body->Void) {
	enteredEvents.push(event);
    }

    public function attachOnExited(event:Body->Void) {
	exitedEvents.push(event);
    }

    public function removeOnEntered(event:Body->Void) {
	enteredEvents.remove(event);
    }

    public function removeOnExited(event:Body->Void) {
	exitedEvents.remove(event);
    }
    
    private function collisionEntered(b:Body) {
	for(event in enteredEvents) {
	    event(b);
	}
    }

    private function collisionExited(b:Body) {
	for(event in exitedEvents) {
	    event(b);
	}
    }

    
    
    public function new() {
	cx = 0;
	cy = 0;
	xr = 0.0;
	yr = 0.0;
	xx = 0.0;
	yy = 0.0;
	dx = 0.0;
	dy = 0.0;
	gravityScale = 1.0;
	radius = 1.0;
	restitution = 0.0;

	new_collisions = new Array<Body>();
        collisions = new Array<Body>();
	old_collisions = new Array<Body>();

	exitedEvents = new Array<Body->Void>();
	enteredEvents = new Array<Body->Void>();
    }

    public inline function overlaps(b:Body):Bool {
	var maxDist = radius + b.radius;

	var distSqr = (b.xx-xx)*(b.xx-xx) + (b.yy-yy)*(b.yy-yy);
	return distSqr<=maxDist*maxDist;
    }

    public function hasCollision(cx:Int, cy:Int):Bool {
	return false;
    }
    
    private function handleX(dt:Float, gravity:Float) {
	xr += dx*dt;
	dx += gravity*gravityScale;
	dx *= 0.96;
	
	if(hasCollision(cx+1, cy) && xr>=0.7) {
	    xr = 0.7;
	    dx = 0;
	}
	if(hasCollision(cx-1, cy) && xr<=0.3) {
	    xr = 0.3;
	    dx = 0;
	}
	
	while(xr>1) { xr --; cx ++;}
	while(xr<0) { xr ++; cx --;}
    }

    private function handleY(dt:Float, gravity:Float) {
	yr += dy*dt;
	dy += gravity*gravityScale;
	dy *= 0.96;
	
	if(hasCollision(cx, cy-1) && yr<=0.3) {
	    yr = 0.3;
	    dy = 0;
	}
	if(hasCollision(cx, cy+1) && yr>=0.5) {
	    yr = 0.5;
	    dy = 0;
	}

	while(yr>1) { yr --; cy ++;}
	while(yr<0) { yr ++; cy --;}
    }
    
    public function update(dt:Float, gravity:Vec2) {
	handleX(dt, gravity.x);
	handleY(dt, gravity.y);
	
	xx = Std.int((cx+xr) * PhysicsManager.cell_size);
	yy = Std.int((cy+yr) * PhysicsManager.cell_size);
    }

    public function fixNewCollisions() {
	for(body in old_collisions) {
	    collisionExited(body);
	    old_collisions.remove(body);
	}
	
	for(body in new_collisions) {
	    collisionEntered(body);
	    collisions.push(body);
	    new_collisions.remove(body);
	}
    }

    public function collide(b:Body) {
	if(b != this) {
	    if(Math.abs(cx-b.cx) - (radius+b.radius) <= 2 &&
	       Math.abs(cy-b.cy) - (radius+b.radius) <= 2) {
		var dist = Math.sqrt((b.xx-xx)*(b.xx-xx) + (b.yy-yy)*(b.yy-yy));
		if(dist == 0.0) {
		    dist = 0.001;
		}
		if(dist <= radius+b.radius) {
		    if(!collisions.contains(b)) {
			new_collisions.push(b);
		    }
		    
		    if(!isSensor && !b.isSensor) {
			var force = 1.0;
			var repelPower = (radius + b.radius - dist)/(radius+b.radius)*restitution;
			var shiftPower = (radius + b.radius - dist);
			
			if(!isStatic) {
			    dx -= (b.xx-xx)/dist * repelPower * force;
			    dy -= (b.yy-yy)/dist * repelPower * force;
			    setPos(xx - (b.xx-xx)/dist * shiftPower,
				   yy - (b.yy-yy)/dist * shiftPower);
			}
			if(!b.isStatic) {
			    b.dx += (b.xx-xx)/dist * repelPower * force;
			    b.dy += (b.yy-yy)/dist * repelPower * force;
			}
		    }
		    return;
		}
	    }

	    if(collisions.contains(b)) {
		old_collisions.push(b);
		collisions.remove(b);
	    }
	}
    }
    
    public function setPos(x,y) {
	xx = x;
	yy = y;
	cx = Std.int(xx/PhysicsManager.cell_size);
	cy = Std.int(yy/PhysicsManager.cell_size);
	xr = (xx-cx*PhysicsManager.cell_size) / PhysicsManager.cell_size;
	yr = (yy-cy*PhysicsManager.cell_size) / PhysicsManager.cell_size;
    }
}
