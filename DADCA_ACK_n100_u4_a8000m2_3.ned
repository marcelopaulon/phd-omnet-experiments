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

network DADCA_ACK_n100_u4_a8000m2_3
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
        @display("i=misc/sensor;p=1376.14,7811.25");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1840.31,7900.35");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2195.13,7811.98");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2732.71,7928.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3896.20,7854.91");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4435.09,7959.67");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5130.72,7989.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=7.70,7643.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=358.61,7563.48");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3224.62,7694.32");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4243.90,7653.05");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4797.91,7740.22");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5620.52,7591.47");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1108.38,7364.56");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1854.03,7385.37");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2357.97,7369.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3623.17,7357.95");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5143.92,7504.69");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=88.57,7191.52");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=779.35,7172.26");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1568.87,7092.62");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2729.44,7244.87");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3322.79,7054.47");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4135.65,7171.15");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4686.74,7149.73");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=348.85,6778.12");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=969.83,6832.37");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1947.52,6818.46");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2969.09,6870.58");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3630.52,6769.77");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5272.92,6907.23");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5884.81,6810.32");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2360.59,6710.89");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4071.40,6641.39");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4513.80,6666.62");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=186.90,6459.05");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=956.45,6404.25");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1407.15,6441.77");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1942.58,6276.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2475.09,6308.77");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2848.98,6280.75");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3319.74,6362.65");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3772.99,6312.06");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4968.54,6504.37");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5434.93,6290.87");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5815.72,6462.98");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1121.11,6029.36");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4110.43,6183.37");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=95.43,5999.67");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=576.51,5981.96");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1732.36,5979.84");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2118.45,5858.75");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2782.98,5880.39");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3380.38,6000.66");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4577.31,6005.53");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5090.49,6008.80");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=297.47,5625.09");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1505.85,5702.51");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1851.18,5553.90");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3665.61,5639.09");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4075.61,5527.40");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5656.01,5627.41");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5.68,5347.32");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=970.76,5489.74");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2202.55,5512.17");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2658.94,5369.90");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3284.94,5489.33");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4541.50,5490.86");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5143.70,5490.81");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=579.19,5174.36");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1433.80,5168.72");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1776.11,5072.17");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3646.12,5041.06");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4133.02,5141.99");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4601.11,5112.15");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5929.35,5066.40");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=258.93,4910.89");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=928.71,4958.11");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2510.78,4969.28");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4875.75,4845.55");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5338.33,4997.34");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=619.41,4647.42");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1113.19,4642.19");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1513.64,4651.39");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1923.69,4672.41");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2943.11,4693.15");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3299.34,4695.10");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4196.31,4762.76");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5110.99,4575.12");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=254.98,4366.52");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1312.01,4348.16");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2279.63,4517.03");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3077.77,4308.28");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3569.78,4311.37");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3965.80,4493.18");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4324.89,4397.78");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5724.03,4355.34");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=763.21,4117.45");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1808.29,4237.64");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2488.53,4206.18");
    }

}

