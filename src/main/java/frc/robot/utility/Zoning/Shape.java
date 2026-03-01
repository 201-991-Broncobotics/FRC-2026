package frc.robot.utility.Zoning;

import edu.wpi.first.math.geometry.Translation2d;


/**
 * This is to normalize driving and make it easier for Mael to not drive into people. 
 * It also determines which driver controller/joystick should take priority and allows them to be used interchangeably
 */

public abstract class Shape {
    public enum ShapeType{
        RECTANGLE,
        CIRCLE,
        SEMICIRCLE
    };

    Shape(){}

    protected Translation2d _center;
    protected double _height, _width; 
    protected ShapeType _shapeType;

    public Translation2d getCenterPoint(){
        return this._center;
    }
    public double getHeight(){
        return this._height;
    }
    public double getWidth(){
        return this._width;
    }
    public ShapeType getShapeType(){
        return this._shapeType;
    }

    public abstract boolean inArea(Translation2d pose);

    //All the different Shapes
    public static class Square extends Shape{
        public Square(Translation2d center, double width){
            this._center = center;
            this._width = width;
            this._height = width;

            this._shapeType = ShapeType.RECTANGLE;
        }

        @Override
        public boolean inArea(Translation2d pose){
            if (Math.abs(this._center.getX() - pose.getX()) < this._width/2 && Math.abs(this._center.getY() - pose.getY()) < this._height/2) {
                return true;
            } else {
                return false;
            }
        }
    }

    public static class Rectangle extends Shape{
        public Rectangle(Translation2d center, double width, double height){
            this._center = center;
            this._height = height;
            this._width = width;

            this._shapeType = ShapeType.RECTANGLE;
        }

        public Rectangle(Translation2d topLeftPoint, Translation2d bottomRightPoint){
            this._center = new Translation2d(
        (topLeftPoint.getX() + bottomRightPoint.getX()) / 2.0, 
        (topLeftPoint.getY() + bottomRightPoint.getY()) / 2.0
    );
            this._height = topLeftPoint.getY() - bottomRightPoint.getY();
            this._width = bottomRightPoint.getX()-topLeftPoint.getX();

            this._shapeType = ShapeType.RECTANGLE;
        }

        @Override
        public boolean inArea(Translation2d pose){
            if (Math.abs(this._center.getX() -  pose.getX()) < this._width/2 && Math.abs(this._center.getY() -  pose.getY()) < this._height/2) {
                return true;
            } else {
                return false;
            }
        }
    }

    public static class Circle extends Shape{
        public Circle(Translation2d center, double radius){
            this._center = center;
            this._height = radius;
            this._width = radius;

            this._shapeType = ShapeType.CIRCLE;
        }

        @Override
        public boolean inArea(Translation2d pose){
            if (this._center.getDistance(pose) < this._width) {
                return true;
            } else {
                return false;
            }
        }
    }

    public static class Semicircle extends Shape{
        private double _startAngle, _endAngle;

        public Semicircle(Translation2d center, double radius, double startAngle, double endAngle){
            this._center = center;
            this._height = radius;
            this._width = radius;

            this._startAngle = startAngle;
            this._endAngle = endAngle;

            this._shapeType = ShapeType.SEMICIRCLE;
        }

        @Override
        public boolean inArea(Translation2d pose){
            double angleFromCenter = Math.atan2(pose.getY()-this._center.getY(), pose.getX()-this._center.getX());

            if (this._center.getDistance(pose) < this._width && (angleFromCenter > _startAngle && angleFromCenter < _endAngle)) {
                return true;
            } else {
                return false;
            }
        }
    }
}