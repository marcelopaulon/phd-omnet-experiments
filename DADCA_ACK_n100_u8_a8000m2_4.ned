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

network DADCA_ACK_n100_u8_a8000m2_4
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
        @display("i=misc/sensor;p=974.08,7921.78");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1660.25,7951.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2046.82,7860.65");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2982.02,7786.88");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3559.59,7846.08");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4630.80,7997.36");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5667.11,7930.06");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=681.39,7708.81");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1392.80,7662.18");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2514.18,7648.56");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4062.85,7685.88");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5830.91,7585.26");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=92.12,7303.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1797.70,7473.10");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2611.40,7262.09");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3046.04,7349.51");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4663.05,7438.22");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5276.34,7329.17");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=500.13,7208.32");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1052.27,7109.71");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1446.61,7126.19");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2230.97,7075.55");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3497.48,7079.01");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4135.48,7212.47");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4728.64,7075.41");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5908.27,7077.96");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=298.70,6845.49");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1838.30,6895.73");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2940.40,6767.97");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4335.92,6916.27");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=738.07,6600.62");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1100.53,6671.08");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1629.88,6515.70");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2115.10,6625.20");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2464.59,6730.46");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4034.46,6625.56");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4380.75,6539.93");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4869.23,6588.26");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5625.89,6742.17");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1050.84,6307.41");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2658.72,6361.41");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3698.88,6450.63");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5258.66,6293.43");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=722.49,6131.83");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1458.63,6169.85");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2232.16,6145.62");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3184.10,6140.24");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4070.31,6229.47");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4408.24,6058.47");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4795.58,6038.57");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5734.12,6223.91");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=357.10,5930.50");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1184.15,5841.30");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1695.62,5857.51");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2697.67,6007.62");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3745.78,5797.46");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5639.34,5859.67");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=700.02,5737.77");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2055.29,5661.77");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2829.54,5642.30");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4014.99,5570.76");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4660.91,5686.43");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4998.36,5547.44");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5355.23,5557.50");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=276.32,5290.15");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=755.36,5350.96");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1730.00,5432.52");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2499.30,5380.11");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3273.35,5484.65");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3611.56,5289.09");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4341.50,5315.73");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5642.90,5351.72");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1390.50,5225.05");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1931.62,5139.86");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4004.06,5121.60");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4700.32,5084.79");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5121.94,5113.68");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=459.47,4864.69");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=830.21,4789.63");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1503.57,4802.77");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2863.01,4990.05");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5730.85,4985.37");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2.86,4664.68");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2269.04,4568.17");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2940.38,4636.71");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3329.83,4714.00");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3714.84,4546.00");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4334.57,4768.72");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4923.18,4642.65");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5334.73,4745.35");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5966.37,4656.36");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1386.46,4445.68");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4058.34,4423.75");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4559.76,4370.96");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=32.57,4283.50");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=409.37,4185.69");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1800.02,4275.20");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2356.06,4161.28");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2813.84,4180.13");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3298.14,4227.89");
    }

}

