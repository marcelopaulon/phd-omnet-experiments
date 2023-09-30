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

network DADCA_ACK_n100_u4_a8000m2_7
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
        @display("i=misc/sensor;p=1435.76,7876.53");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1961.90,7920.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3537.91,7933.57");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3900.53,7921.83");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4368.97,7802.40");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5382.57,7771.57");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5873.76,7817.00");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=23.62,7692.11");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=609.48,7611.63");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1674.97,7591.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2570.76,7694.50");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3002.24,7599.03");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3762.09,7544.49");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4964.97,7701.72");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2789.94,7269.67");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3269.55,7366.04");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4551.58,7411.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5282.25,7329.91");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5831.30,7382.91");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=380.11,7204.79");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1026.46,7083.55");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2285.87,7113.88");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4061.16,7198.83");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4934.37,7167.14");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5596.21,7091.59");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=662.44,6994.73");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1610.68,6987.80");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2815.81,6775.40");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3458.01,6984.45");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4477.24,6906.11");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4833.74,6805.93");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=78.23,6696.64");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1357.21,6646.96");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3972.15,6639.90");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5412.38,6671.60");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=151.89,6326.95");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=492.59,6506.42");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=957.21,6475.88");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1938.01,6454.21");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2427.07,6504.25");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3037.06,6355.18");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3572.75,6506.23");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4185.33,6293.44");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4546.11,6481.40");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4888.89,6357.34");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5996.17,6407.15");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=521.60,6046.39");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1284.64,6208.65");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2230.08,6189.50");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2673.54,6102.56");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3794.51,6103.52");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5499.60,6197.73");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5857.27,6048.34");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=43.88,5973.75");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1264.50,5854.64");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1630.87,5853.23");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2481.73,5785.54");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3332.02,5955.01");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4649.90,5910.84");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=64.88,5622.82");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1137.59,5527.17");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2061.68,5772.20");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3761.34,5754.54");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5339.68,5752.25");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=360.20,5354.55");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1488.67,5476.63");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2795.72,5512.07");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3137.38,5302.95");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3772.47,5290.00");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4112.44,5472.30");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5074.97,5438.51");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5675.38,5496.37");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=994.85,5168.66");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1886.36,5270.77");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2502.12,5218.60");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4117.25,5046.38");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4704.99,5220.60");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5451.40,5189.96");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=501.84,4816.00");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1473.30,4971.09");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2977.15,4892.00");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3660.90,4941.11");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4524.19,4912.78");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5843.43,4795.24");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=55.93,4766.09");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=883.49,4717.62");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1664.66,4583.38");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2330.28,4728.52");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4029.56,4669.22");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5084.09,4778.21");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5412.63,4617.90");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=574.77,4444.12");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1321.18,4399.86");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1961.74,4353.21");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2505.87,4422.14");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3212.17,4396.76");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3632.89,4469.32");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4885.71,4449.01");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5927.76,4390.30");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=285.83,4188.28");
    }

}

