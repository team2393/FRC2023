package frc.robot.tutorial;

public class Karma
{
    protected int points;

    public Karma()
    {
        do_something_bad();
    }

    public void do_something_good()
    {
        ++points;
    }

    public int getPoints()
    {
        return points;
    }

    public double get_money()
    {
        return points * 0.00000000000001;
    }
    
    public void do_something_bad()
    {
        points = 0;
    }
}
