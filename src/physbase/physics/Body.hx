package physbase.physics;

import physbase.maths.Types;
import physbase.physics.*;

enum BodyType {
    Circle(r:Float);
    Rect(w:Float,h:Float);
}

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

    public final aabb:AABB;
    
    public var type:BodyType;
    public var mass : Float;

    
    public final new_collisions : Array<Body>;
    public final collisions : Array<Body>;
    public final old_collisions : Array<Body>;

    public final enteredEvents : Array<Body->Void>;
    public final exitedEvents : Array<Body->Void>;

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


    public function aabbFromPixels(w:Float, h:Float) {
        this.aabb.xMin = 0.0;
	this.aabb.xMax = w/PhysicsManager.cell_size;
	this.aabb.yMin = 0.0;
	this.aabb.yMax = h/PhysicsManager.cell_size;
    }
    
    public function radius() {
	switch(type) {
	case Rect(w, h):
	    return Math.min(w,h);
	case Circle(r):
	    return r;
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
	type = Circle(1.0);
	aabb = {
	    xMin:0.0, xMax: 1.0,
	    yMin:0.0, yMax: 1.0
	};  
	restitution = 0.0;
	mass = 1.0;

	new_collisions = new Array<Body>();
        collisions = new Array<Body>();
	old_collisions = new Array<Body>();

	exitedEvents = new Array<Body->Void>();
	enteredEvents = new Array<Body->Void>();
    }

    public inline function overlaps(b:Body):Bool {
        final maxDist = radius() + b.radius();

        final distSqr = (b.xx-xx)*(b.xx-xx) + (b.yy-yy)*(b.yy-yy);
	return distSqr<=maxDist*maxDist;
    }

    public dynamic function hasCollision(cx:Float, cy:Float):Bool {
	return false;
    }

    private function aabbMaxTrue(max:Float):Float {
	return max < 1.0 ? max : max%1.0;
    }

    private function posToCell(value:Float) {
	return value/PhysicsManager.cell_size;
    }
    
    private function handleX(dt:Float, gravity:Float) {
	xr += dx*dt;
	dx += gravity*gravityScale;
	//dx *= 0.96;
	if(dx >= 0 &&
	   (hasCollision(cx + aabb.xMax + xr,
			 cy + yr + aabb.yMin + PhysicsManager.half_pixel)||
	    hasCollision(cx + aabb.xMax + xr,
			 cy + yr + aabb.yMax - PhysicsManager.half_pixel))
	   && xr>=(1.0-aabbMaxTrue(aabb.xMax))) {
	    xr = (1.0-aabbMaxTrue(aabb.xMax));
	    dx = 0;
	}
	if(dx <= 0 &&
	   (hasCollision(cx + aabb.xMin - (aabb.xMin >= 0.0 ? 1 : 0),
			 cy + yr + aabb.yMin + PhysicsManager.half_pixel) ||
	    hasCollision(cx + aabb.xMin - (aabb.xMin >= 0.0 ? 1 : 0),
			 cy + yr + aabb.yMax - PhysicsManager.half_pixel))
	   && xr<=aabb.xMin) {
	    xr = aabb.xMin;
	    dx = 0;
	}
	
	while(xr>1) { xr --; cx ++;}
	while(xr<0) { xr ++; cx --;}
    }

    private function handleY(dt:Float, gravity:Float) {
	yr += dy*dt;
	dy += gravity*gravityScale;
	//dy *= 0.96;
	if(dy >= 0 &&
	   (hasCollision(cx + xr + aabb.xMin + PhysicsManager.half_pixel,
			 cy + aabb.yMax + yr)||
	    hasCollision(cx + xr + aabb.xMax - PhysicsManager.half_pixel,
			 cy + aabb.yMax + yr))
	   && yr>=(1.0-aabbMaxTrue(aabb.yMax))) {
	    yr = (1.0-aabbMaxTrue(aabb.yMax));
	    dy = 0;
	}
	if(dy <= 0 &&
	   (hasCollision(cx + xr + aabb.xMin + PhysicsManager.half_pixel,
			 cy + aabb.yMin - (aabb.yMin >= 0.0 ? 1 : 0)) ||
	    hasCollision(cx + xr + aabb.xMax - PhysicsManager.half_pixel,
			 cy + aabb.yMin - (aabb.yMin >= 0.0 ? 1 : 0)))
	   && yr<=aabb.yMin) {
	    yr = aabb.yMin;
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

    public function fixPosition() {
	xx = Std.int((cx+xr) * PhysicsManager.cell_size);
	yy = Std.int((cy+yr) * PhysicsManager.cell_size);
    }

    public function fixCollisionState(b:Body) {
	if(collisions.contains(b)) {
	    old_collisions.push(b);
	    collisions.remove(b);
	}
    }
    
    public function collide(b:Body) {
	if(b != this) {
	    if(type.getName() == "Rect" ||
	       b.type.getName() == "Rect") {
		if(!collideRect(b)) {
		    fixCollisionState(b);
		}
	    }
	    else {
		if(!collideCirc(b)) {
		    fixCollisionState(b);
		}
	    }
	}
	
    }

    private function typeToRect(t:BodyType):BodyType {
	switch(t) {
	case Circle(r):
	    return Rect(r*2, r*2);
	default:
	    return t;
	}
	
    }

    private function hasCollidedRect(x1:Float, y1:Float,
				     w1:Float,h1:Float,
				     x2:Float, y2:Float,
				     w2:Float,h2:Float):Bool {
	return x1 < x2 + w2 &&
	    x1 + w1 > x2 &&
	    y1 < y2 + h2 &&
	    y1 + h1 > y2;
    }

    // Distance from Axis and Length
    private function distFromAL(axis1:Float, axis2:Float,
				length1:Float, length2:Float):Float {
	if(axis1 < axis2) {
	    return axis2 - (axis1 + length1);
	}
	else if(axis1 > axis2) {
	    return axis1 - (axis2 + length2);
	}
	return 0.0;   
    }
    
    private function AABBDistance(x1:Float, y1:Float,
				  w1:Float,h1:Float,
				  x2:Float, y2:Float,
				  w2:Float,h2:Float) {
        final distx = distFromAL(x1, x2, w1, w2);
        final disty = distFromAL(y1, y2, h1, h2);

	return {x:distx, y:disty};
    }

    private inline function floorPixel(value:Float):Float {
	return Std.int(value*PhysicsManager.cell_size)/PhysicsManager.cell_size;
    }
    
    private function collideRect(b:Body):Bool {
        final tt = typeToRect(type);
        final bt = typeToRect(b.type);

	switch(tt) {
	case Rect(ww, hh):
	    switch(bt) {
	    case Rect(bw, bh):

		final old_dx = dx;
		final old_dy = dy;
		
		if(!isSensor && !b.isSensor &&
		   hasCollidedRect(xx, yy, ww, hh,
				   b.xx, b.yy, bw, bh)) {

		    if(!collisions.contains(b)) {
			new_collisions.push(b);
		    }
		    
		    final dist = AABBDistance(xx, yy, ww, hh,
					      b.xx, b.yy, bw, bh);

		    final xAxisTime:Float = dx != 0 ?
			(Math.abs(dist.x / dx)/PhysicsManager.cell_size)
			: 0.0;
		    final yAxisTime:Float = dy != 0 ?
			(Math.abs(dist.y / dy)/PhysicsManager.cell_size)
			: 0.0;

		    
		    if(dx != 0 && dy == 0) {
		        // Fix Position
			xr -= dx*xAxisTime;
		    }
		    else if(dx == 0 && dy != 0) {
			// Fix Position
			yr -= dy*yAxisTime;
		    }
		    else {
			// Fix Position
			final shortestTime = Math.min(Math.abs(xAxisTime),
						      Math.abs(yAxisTime));
			xr -= dx*shortestTime;
		        yr -= dy*shortestTime;
		    }

		    xr = floorPixel(xr);
		    yr = floorPixel(yr);
		    
		    // Implement Collision Stuff Here
		    return true;
		}
			
	    default:
	    }
	default:
	}
	return false;
    }
    
    private function collideCirc(b:Body):Bool {
	
	if(Math.abs(cx-b.cx) - (radius()+b.radius()) <= 2 &&
	   Math.abs(cy-b.cy) - (radius()+b.radius()) <= 2) {
	    final dist : Float = (b.xx == xx && b.yy == yy) ?
		0.0 : Math.sqrt((b.xx-xx)*(b.xx-xx) + (b.yy-yy)*(b.yy-yy));
	    if(dist <= radius()+b.radius()) {
		if(!collisions.contains(b)) {
		    new_collisions.push(b);
		}
		    
		if(!isSensor && !b.isSensor) {
		    final force = 1.0;
		    final repelPower = (radius() + b.radius() - dist)/(radius()+b.radius())*restitution;
			
		    final shiftPower = (radius() + b.radius() - dist);
			
		    if(!isStatic) {
			dx -= (b.xx-xx)/dist * repelPower * force;
			dy -= (b.yy-yy)/dist * repelPower * force;

			    
		        final x = cellToPos(cx, xr);
		        final y = cellToPos(cy, yr);
		        final bx = cellToPos(b.cx, b.xr);
		        final by = cellToPos(b.cy, b.yr);

			setCellPos(x - (bx-x)/dist * shiftPower,
				   y - (by-y)/dist * shiftPower);
		    }
		    if(!b.isStatic) {
			    
			b.dx += (b.xx-xx)/dist * repelPower * force;
			b.dy += (b.yy-yy)/dist * repelPower * force;
		    }
		}
		return true;
	    }
	}
	return false;
    }

    function cellToPos(cell:Int, ratio:Float):Float {
	return cell*PhysicsManager.cell_size + ratio*PhysicsManager.cell_size;
    }
    
    function setCellPos(x:Float, y:Float) {
	cx = Std.int(x/PhysicsManager.cell_size);
	cy = Std.int(y/PhysicsManager.cell_size);
	xr = (x-cx*PhysicsManager.cell_size) / PhysicsManager.cell_size;
	yr = (y-cy*PhysicsManager.cell_size) / PhysicsManager.cell_size;
    }
    
    public function setPos(x,y) {
	xx = x;
	yy = y;
	setCellPos(x, y);
    }
}


typedef AABB = {
    var xMin:Float;
    var yMin:Float;
    var xMax:Float;
    var yMax:Float;
}
