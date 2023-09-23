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

network DADCA_ACK_n100_u3_a8000m2_4
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
        @display("i=misc/sensor;p=402.63,7782.49");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=760.85,7847.84");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1173.36,7966.95");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1633.17,7953.09");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2555.09,7976.43");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3154.78,7769.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3592.01,7881.11");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4065.08,7863.80");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4687.20,7897.12");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5311.58,7933.50");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5750.59,7863.86");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1472.00,7534.14");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1841.97,7506.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2172.92,7745.94");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4522.93,7582.09");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=915.63,7398.74");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2468.01,7498.31");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3377.63,7483.39");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3762.69,7451.65");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5045.39,7413.61");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5978.67,7503.96");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1259.86,7134.95");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2086.17,7099.10");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2441.82,7038.55");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3015.33,7148.87");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3489.33,7029.50");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4133.24,7238.51");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5555.38,7197.13");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=704.40,6895.82");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1661.30,6958.21");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4390.80,6956.88");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5415.24,6803.62");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5896.03,6997.94");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1079.61,6527.54");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1899.61,6642.28");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2540.42,6702.72");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2902.28,6605.86");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3658.71,6680.30");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=217.54,6456.80");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1455.63,6307.27");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1934.85,6270.00");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2539.64,6327.98");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3245.44,6414.43");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4455.82,6370.46");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4797.85,6501.86");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5646.71,6472.71");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=602.01,6059.05");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=987.48,6119.35");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3652.72,6030.47");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5249.16,6118.87");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5608.92,6092.44");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2275.18,5913.55");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2714.66,5920.98");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3362.32,5787.07");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4021.51,5879.33");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4495.95,5849.97");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5936.23,5849.13");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=348.87,5715.44");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1643.14,5677.36");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2178.10,5534.71");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2947.50,5590.23");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4762.95,5602.90");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5451.69,5761.98");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=52.19,5447.36");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=755.63,5490.12");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1228.19,5466.04");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2592.30,5372.69");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3976.52,5495.09");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=448.46,5155.48");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1742.38,5202.36");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2169.64,5081.10");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2809.81,5048.94");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3494.67,5160.10");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4039.66,5032.86");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4538.23,5151.34");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5886.50,5215.63");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=24.38,4975.88");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1079.33,4967.21");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3183.00,4969.36");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3702.32,4787.18");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4853.20,4929.00");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5226.70,4959.49");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=559.78,4608.42");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1194.43,4609.40");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1640.29,4598.59");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2281.14,4699.49");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2719.81,4593.77");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3398.32,4575.76");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4597.36,4678.37");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5822.73,4552.13");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=239.59,4460.80");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2044.11,4388.20");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4053.95,4360.83");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4981.20,4399.77");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=496.34,4212.60");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=876.51,4283.42");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1144.98,4053.32");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1428.49,4263.00");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3057.65,4172.20");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3698.90,4236.12");
    }

}

