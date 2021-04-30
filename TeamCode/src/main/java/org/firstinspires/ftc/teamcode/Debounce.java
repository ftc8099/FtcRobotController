package org.firstinspires.ftc.teamcode;

public class Debounce {
    private int counter = 0;
    private boolean last = false;
    private boolean old = false;
    private int minCount;

    Debounce(int minCount){
        this.minCount = minCount;
    }

    public int getCount(){
        return counter;
    }

    public boolean getLast(){
        return last;
    }

    public boolean clean(boolean button){
        if (last != button){
            counter = 0;
        }
        else {
            counter++;
        }
        last = button;
        if (counter >= minCount){
            old = last;
            return button;
        }
        else {
            return old;
        }
    }

    public boolean press(boolean button){
        if (last != button){
            counter = 0;
        }
        else {
            counter++;
        }
        last = button;
        if (counter == minCount && button == true){
            return true;
        }
        else {
            return false;
        }
    }
}
