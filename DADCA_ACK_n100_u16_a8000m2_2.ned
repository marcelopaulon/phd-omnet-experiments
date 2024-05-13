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

network DADCA_ACK_n100_u16_a8000m2_2
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
        @display("i=misc/sensor;p=488.45,7759.85");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1171.64,7898.80");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2726.09,7923.97");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3304.18,7773.59");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3874.28,7925.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4346.40,7764.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5017.07,7798.19");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5537.18,7914.89");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=38.53,7568.96");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=886.95,7686.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1566.69,7637.76");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2117.44,7605.90");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=398.30,7383.37");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2668.55,7491.83");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3249.54,7345.47");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3752.78,7334.05");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4189.09,7451.57");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4543.65,7467.12");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4906.77,7394.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5365.75,7497.58");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5910.79,7377.34");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=662.93,7137.97");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1154.64,7151.16");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1923.46,7256.43");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4385.14,7076.75");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4773.40,7047.40");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5580.58,7068.00");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=206.57,6965.53");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1664.81,6994.36");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2464.48,6793.51");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2869.80,6905.37");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3430.31,6775.74");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3955.60,6918.31");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5160.99,6908.44");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=586.12,6602.91");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1208.22,6676.84");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2138.88,6654.55");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4570.04,6604.62");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5698.04,6721.04");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=93.05,6421.14");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=878.14,6317.99");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1388.62,6354.67");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1783.32,6436.14");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2150.68,6295.24");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2684.99,6444.13");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3051.67,6505.00");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3707.51,6275.05");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4127.04,6436.09");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4977.03,6398.36");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5555.99,6271.10");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=534.51,6140.91");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2749.94,6078.59");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3263.11,6222.52");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4220.04,6040.17");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=414.09,5793.43");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1029.30,5924.36");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1515.41,5971.78");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1880.38,5979.10");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4581.91,5945.04");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5005.58,5979.17");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5657.08,5926.73");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=19.93,5609.76");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1356.36,5583.18");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1757.24,5581.49");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2224.51,5688.49");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2825.50,5672.81");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3560.72,5656.57");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4248.25,5625.97");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4796.99,5610.79");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5510.83,5572.74");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=967.92,5455.32");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2031.28,5281.04");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5910.71,5509.44");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=446.73,5258.43");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=994.06,5041.72");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1489.46,5216.37");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2657.21,5172.51");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3101.73,5197.26");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3831.01,5153.10");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5006.53,5177.48");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5477.97,5175.98");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1344.80,4805.15");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1974.29,4929.63");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2328.12,4994.18");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3492.59,4902.40");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4404.27,4964.05");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5257.64,4854.17");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5855.81,4982.00");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2356.85,4582.39");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2872.70,4716.44");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3908.35,4697.60");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4428.27,4589.73");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4824.89,4584.30");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5630.65,4678.83");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5970.03,4583.70");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1148.44,4341.20");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1901.39,4496.94");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3394.85,4392.44");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5352.93,4325.98");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=215.08,4157.62");
    }

}

