import math
import unittest
from scan_gap_candidate import fill_single_bin_gaps


class GapFillTest(unittest.TestCase):
    def test_wall_chord_and_originals(self):
        a=math.pi/180
        values=[2/math.cos(a),float('inf'),2/math.cos(a)]
        filled,n=fill_single_bin_gaps(values,a)
        self.assertEqual(n,1);self.assertAlmostEqual(filled[1],2)
        self.assertEqual(filled[0],values[0]);self.assertEqual(filled[2],values[2])
        self.assertTrue(math.isinf(values[1]))

    def test_never_fill_large_gap_edge_or_discontinuity(self):
        for values in [[float('inf'),2,2], [2,2,float('inf')],
                       [2,float('inf'),float('inf'),2], [2,float('inf'),3]]:
            result,n=fill_single_bin_gaps(values,math.pi/180)
            self.assertEqual(n,0);self.assertEqual(result,values)

    def test_no_temporal_state(self):
        fill_single_bin_gaps([2,float('inf'),2],math.pi/180)
        values=[float('inf')]*360
        result,n=fill_single_bin_gaps(values,math.pi/180)
        self.assertEqual(n,0);self.assertEqual(result,values)

    def test_bad_geometry(self):
        for angle in [0,-1,float('nan'),math.pi/90]:
            with self.assertRaises(ValueError): fill_single_bin_gaps([1,1,1],angle)



if __name__=='__main__': unittest.main()
