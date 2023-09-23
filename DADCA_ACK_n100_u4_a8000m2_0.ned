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

network DADCA_ACK_n100_u4_a8000m2_0
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
        @display("i=misc/sensor;p=1467.41,7964.87");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2193.67,7809.23");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2577.80,7761.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3465.82,7838.44");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4136.73,7860.60");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4721.18,7936.63");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5702.79,7907.01");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=872.59,7652.89");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3757.05,7599.64");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4571.01,7545.14");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5099.71,7628.20");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5.07,7489.75");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1167.83,7441.63");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1582.00,7430.16");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2088.30,7428.95");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3074.83,7309.35");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=555.35,7245.58");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2481.03,7178.35");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3535.04,7113.42");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4247.78,7156.79");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4880.73,7076.12");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5504.23,7238.64");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5973.89,7208.45");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=306.09,6875.13");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1190.69,7004.97");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1939.98,6936.31");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4495.29,6844.63");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5430.65,6858.89");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=701.77,6750.71");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1190.55,6553.82");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1631.99,6553.31");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2232.68,6526.10");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2779.82,6745.79");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3207.17,6715.33");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3617.71,6739.09");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2528.13,6307.01");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2900.35,6372.02");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3304.74,6299.08");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4175.19,6493.18");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4936.06,6408.75");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5298.76,6267.90");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5891.44,6513.93");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=318.19,6232.19");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=765.66,6219.31");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1308.35,6028.98");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1825.64,6209.58");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3615.96,6137.68");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3978.08,6189.77");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4392.18,6203.74");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1754.77,5827.86");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2327.51,5994.17");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2891.56,5844.24");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3300.47,5831.66");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5512.38,5820.06");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5883.73,5869.53");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=262.40,5624.81");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2540.58,5713.51");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3897.53,5568.64");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4309.51,5623.34");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4960.23,5768.11");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=723.43,5461.12");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1307.62,5503.55");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1787.64,5327.52");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2197.10,5480.27");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5733.89,5408.47");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=798.96,5037.42");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2834.25,5246.86");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3319.60,5115.12");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4295.15,5166.28");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4661.62,5247.50");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5267.65,5107.29");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=227.44,4991.64");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1205.76,4985.01");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1546.04,4809.93");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2459.10,4947.76");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3046.39,4799.20");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3751.82,4929.63");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5628.86,4892.94");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=30.07,4682.42");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=941.92,4660.33");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2282.67,4599.54");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3838.81,4584.21");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4373.71,4616.14");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5030.00,4779.72");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=271.17,4301.41");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=548.08,4521.09");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1828.42,4478.74");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2676.42,4377.71");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3049.98,4300.73");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4123.00,4357.16");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4580.92,4321.88");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5235.86,4496.48");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5883.86,4297.50");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1129.42,4147.60");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2268.42,4116.40");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3406.80,4237.65");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5493.55,4054.23");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=98.61,3927.01");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=647.81,4017.86");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1477.32,3921.67");
    }

}

