package physbase.maths;


typedef Vec2 = {    
    var x:Float;
    var y:Float;
};

abstract Bint(Int) from Int to Int {
    
    public function new(i:Int) {
	this = i;
    }
    @:from
    public static function fromBool(b:Bool):Bint {
	return new Bint(b ? 1 : 0);
    }
}
