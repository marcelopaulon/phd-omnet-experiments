package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n100_u3_a8000m2_8
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=490.49,7906.38");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1864.32,7860.73");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2999.89,7784.54");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4285.89,7871.98");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5068.46,7925.68");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=165.72,7539.12");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=625.26,7536.28");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=970.07,7657.13");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2432.49,7659.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3578.41,7728.74");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4007.44,7575.07");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4643.65,7618.44");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5394.69,7702.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1362.99,7330.89");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2038.24,7474.44");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2618.57,7322.98");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3382.77,7306.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4999.76,7459.73");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=731.11,7172.80");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1963.74,7093.92");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3026.67,7169.61");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3714.83,7070.42");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4372.20,7200.02");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4829.40,7111.41");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5748.15,7194.31");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=50.75,6915.28");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1390.80,6923.01");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1723.92,6795.00");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3995.78,6787.58");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4348.53,6788.72");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5210.52,6992.73");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=316.93,6674.65");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=710.57,6721.47");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2210.29,6551.26");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2611.45,6679.46");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3001.65,6756.44");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3498.02,6626.96");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4979.25,6648.31");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5438.73,6675.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1153.17,6487.94");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1525.16,6416.38");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2831.70,6392.48");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3825.91,6325.86");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4544.77,6355.62");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5107.96,6311.67");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=396.07,6196.48");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=756.96,6233.77");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2022.73,6044.16");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2373.87,6212.42");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3504.79,6125.01");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4254.41,6157.49");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5567.59,6059.01");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=257.36,5831.18");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1214.79,5925.09");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3243.11,5815.95");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3896.65,5930.01");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5060.65,5921.26");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=895.68,5698.52");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1499.91,5610.61");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2437.06,5670.89");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2885.92,5746.80");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3627.75,5683.32");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4771.60,5690.46");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5918.75,5595.68");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=541.33,5372.63");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1115.05,5309.81");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2000.55,5358.74");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3950.31,5402.96");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4345.96,5511.92");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4944.95,5295.92");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5321.17,5466.06");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=212.02,5175.24");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1726.70,5046.42");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2341.03,5190.98");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2870.56,5229.15");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3399.55,5247.85");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3719.25,5093.92");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5983.18,5218.26");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=762.41,4936.70");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2606.27,4941.73");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4072.58,4950.29");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4711.74,4967.79");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5180.79,4901.05");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=137.69,4705.74");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1260.09,4627.81");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3165.11,4646.20");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3614.18,4648.88");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5934.25,4601.26");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=320.86,4391.59");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=686.46,4369.11");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1997.51,4524.03");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2545.34,4482.38");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3889.86,4359.41");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4288.21,4479.79");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4770.37,4386.73");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5632.45,4377.78");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1245.80,4203.85");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2857.62,4144.77");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=630.97,3929.83");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1069.69,3811.80");
    }

}

