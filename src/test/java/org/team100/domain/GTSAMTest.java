package org.team100.domain;

import org.junit.jupiter.api.Test;

import gtsam.Point2;

public class GTSAMTest {

    @Test
    void testGtsam() throws Throwable {
        Point2 p = new Point2(4, 5);
        System.out.println("Point2 print:");
        p.print();
        System.out.println("Point2 print done!");

    }
}
