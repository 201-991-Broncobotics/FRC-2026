package frc.robot.utility.Zoning;

import edu.wpi.first.math.geometry.Translation2d;

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

    public abstract Translation2d getXYDistanceFromClosestPoint(Translation2d pose);
    public abstract double getDistanceFromX(Translation2d pose);
    public abstract double getDistanceFromY(Translation2d pose);

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

        @Override
        public Translation2d getXYDistanceFromClosestPoint(Translation2d pose) {
            double top = this._center.getY() + this._height / 2.0;
            double bottom = this._center.getY() - this._height / 2.0;
            double left = this._center.getX() - this._width / 2.0;
            double right = this._center.getX() + this._width / 2.0;
            
            // not in axis with any sides
            if (!(pose.getX() < right && pose.getX() > left) && !(pose.getY() < top && pose.getY() > bottom)) {
                return new Translation2d(
                    getDistanceFromX(pose),
                    getDistanceFromY(pose)
                );
            }

            double closestX = closestToZero(left - pose.getX(), right - pose.getX());
            double closestY = closestToZero(bottom - pose.getY(), top - pose.getY());

            if (closestX < closestY) return new Translation2d(closestX, 0);
            else return new Translation2d(0, closestY);
        }

        @Override
        public double getDistanceFromX(Translation2d pose) {
            return closestToZero((this._center.getX() + this._width/2.0) - pose.getX(), (this._center.getX() - this._width/2.0) - pose.getX());
        }

        @Override
        public double getDistanceFromY(Translation2d pose) {
            return closestToZero((this._center.getY() + this._height/2.0) - pose.getY(), (this._center.getY() - this._height/2.0) - pose.getY());
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

        @Override
        public Translation2d getXYDistanceFromClosestPoint(Translation2d pose) {
            double top = this._center.getY() + this._height / 2.0;
            double bottom = this._center.getY() - this._height / 2.0;
            double left = this._center.getX() - this._width / 2.0;
            double right = this._center.getX() + this._width / 2.0;
            
            // not in axis with any sides
            if (!(pose.getX() < right && pose.getX() > left) && !(pose.getY() < top && pose.getY() > bottom)) {
                return new Translation2d(
                    getDistanceFromX(pose),
                    getDistanceFromY(pose)
                );
            }

            double closestX = closestToZero(left - pose.getX(), right - pose.getX());
            double closestY = closestToZero(bottom - pose.getY(), top - pose.getY());

            if (closestX < closestY) return new Translation2d(closestX, 0);
            else return new Translation2d(0, closestY);
        }

        @Override
        public double getDistanceFromX(Translation2d pose) {
            return closestToZero((this._center.getX() + this._width/2.0) - pose.getX(), (this._center.getX() - this._width/2.0) - pose.getX());
        }

        @Override
        public double getDistanceFromY(Translation2d pose) {
            return closestToZero((this._center.getY() + this._height/2.0) - pose.getY(), (this._center.getY() - this._height/2.0) - pose.getY());
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

        @Override
        public Translation2d getXYDistanceFromClosestPoint(Translation2d pose) {
            Translation2d displacement = pose.minus(this._center);
            double angle = Math.atan2(displacement.getY(), displacement.getX());
            return new Translation2d(
                closestToZero(displacement.getX() - Math.cos(angle) * this._width, displacement.getX() + Math.cos(angle) * this._width),
                closestToZero(displacement.getY() - Math.sin(angle) * this._width, displacement.getY() + Math.sin(angle) * this._width)
            );
        }

        @Override
        public double getDistanceFromX(Translation2d pose) {
            return getXYDistanceFromClosestPoint(pose).getX();
        }

        @Override
        public double getDistanceFromY(Translation2d pose) {
            return getXYDistanceFromClosestPoint(pose).getY();
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

        @Override
        public Translation2d getXYDistanceFromClosestPoint(Translation2d pose) { // I gave up and just used ai for this one

            Translation2d displacement = pose.minus(this._center);
            double px = displacement.getX();
            double py = displacement.getY();

            double angle = Math.atan2(py, px);
            double r = this._width;

            // normalize angles to [-pi, pi]
            double start = _startAngle;
            double end = _endAngle;

            if (end < start) {
                end += 2*Math.PI;
                if (angle < start) angle += 2*Math.PI;
            }

            // --- Case 1: closest to arc ---
            if (angle >= start && angle <= end) {

                double cx = Math.cos(angle) * r;
                double cy = Math.sin(angle) * r;

                return new Translation2d(px - cx, py - cy);
            }

            // --- Case 2/3: closest to radial edges ---
            Translation2d startVec = closestPointOnRadial(px, py, start, r);
            Translation2d endVec = closestPointOnRadial(px, py, end, r);

            if (startVec.getNorm() < endVec.getNorm()) return startVec;
            return endVec;
        }

        @Override
        public double getDistanceFromX(Translation2d pose) {
            return getXYDistanceFromClosestPoint(pose).getX();
        }

        @Override
        public double getDistanceFromY(Translation2d pose) {
            return getXYDistanceFromClosestPoint(pose).getY();
        }
    }

    private static double closestToZero(double value1, double value2) {
        if (Math.abs(value1) < Math.abs(value2)) return value1;
        else return value2;
    }

    private static Translation2d closestPointOnRadial(double px, double py, double angle, double radius) {
        double dx = Math.cos(angle);
        double dy = Math.sin(angle);

        // projection of point onto radial line
        double t = px*dx + py*dy;

        // clamp to segment [0, radius]
        t = Math.max(0, Math.min(radius, t));

        double cx = dx * t;
        double cy = dy * t;

        return new Translation2d(px - cx, py - cy);
    }
}