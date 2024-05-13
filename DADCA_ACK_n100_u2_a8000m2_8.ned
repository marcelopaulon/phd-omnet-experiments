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

network DADCA_ACK_n100_u2_a8000m2_8
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
        @display("i=misc/sensor;p=48.96,7782.13");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=727.61,7915.11");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2851.59,7792.80");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3182.97,7943.66");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3718.21,7845.36");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4596.24,7874.87");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5582.24,7866.78");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=485.58,7579.21");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1270.08,7703.03");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2216.54,7668.81");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5077.17,7680.27");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5997.25,7611.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=195.14,7276.14");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=865.00,7477.34");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1549.87,7352.43");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3134.88,7458.54");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3679.05,7483.38");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4115.08,7476.76");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4680.38,7425.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5327.84,7369.41");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1060.21,7094.36");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2184.83,7141.84");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2839.54,7251.10");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3420.60,7181.02");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4679.65,7072.45");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5782.21,7237.09");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=476.29,6898.55");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1511.47,6888.72");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2406.29,6866.69");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2757.68,6909.18");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3295.72,6823.23");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3702.26,6844.05");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4119.50,6900.84");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5368.50,6844.40");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=778.83,6619.39");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1247.80,6536.67");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1988.18,6522.50");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2600.72,6538.43");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4722.39,6661.10");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5852.66,6727.71");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=211.35,6358.24");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3252.24,6273.45");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3908.71,6293.35");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4398.55,6500.63");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4902.77,6351.03");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=475.77,6099.30");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=986.69,6224.96");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1477.72,6192.52");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1834.90,6143.75");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2378.49,6186.12");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4222.79,6025.20");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5612.26,6260.90");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=805.58,5919.36");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1639.92,5802.65");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2600.48,5837.40");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3128.25,5850.17");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4986.93,5895.38");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5518.86,5895.38");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1042.44,5590.07");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1979.78,5631.78");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3591.30,5768.90");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3952.43,5673.83");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=207.64,5502.80");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=722.81,5345.54");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1394.13,5508.11");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2498.65,5316.42");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2798.69,5507.69");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3249.43,5464.47");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4469.51,5395.85");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5077.02,5387.80");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=950.23,5042.22");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1502.46,5169.48");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2143.70,5151.59");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3533.29,5244.37");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5803.86,5276.42");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=246.07,4935.33");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1346.12,4819.31");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1792.79,4810.61");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2966.50,4926.74");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3671.23,4878.99");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4032.52,5029.20");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4628.87,4899.43");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5474.38,4997.00");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=612.59,4661.00");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3261.41,4718.08");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5758.63,4665.85");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=934.48,4444.62");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1787.05,4364.68");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2814.77,4332.89");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3182.29,4319.75");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3829.60,4510.13");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4248.31,4505.32");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5029.28,4444.05");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=18.02,4273.62");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=487.67,4282.45");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1428.87,4117.11");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2172.41,4247.19");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4638.98,4271.20");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5331.84,4228.01");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=687.11,3958.96");
    }

}

