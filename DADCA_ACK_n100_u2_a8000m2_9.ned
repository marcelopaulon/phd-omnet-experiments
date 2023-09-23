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

network DADCA_ACK_n100_u2_a8000m2_9
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
        @display("i=misc/sensor;p=1290.34,7994.68");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2155.79,7773.40");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2614.87,7765.39");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2990.05,7910.10");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3387.60,7849.84");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5102.73,7927.33");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5564.05,7769.35");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=258.22,7538.56");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=837.85,7735.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1665.02,7563.46");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3510.34,7509.86");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4079.56,7695.53");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5006.88,7576.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5901.73,7667.78");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=675.88,7390.74");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1264.13,7354.86");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2160.42,7404.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2797.79,7421.58");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3873.13,7405.10");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4423.07,7370.95");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5399.82,7442.75");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1037.87,7080.08");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1577.58,7072.54");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2021.71,7026.46");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2622.61,7014.09");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3083.15,7169.57");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3462.45,7109.05");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3899.90,7012.80");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4822.89,7190.23");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5493.69,7101.29");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=259.47,6930.49");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5127.05,6924.37");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=45.74,6630.67");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=668.78,6612.66");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1238.78,6610.02");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1795.14,6545.38");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2578.25,6518.10");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3429.76,6599.11");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3910.44,6636.98");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4472.68,6676.29");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4858.93,6553.00");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2228.04,6389.92");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3156.38,6367.70");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5216.69,6492.25");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5601.92,6497.28");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=275.19,6113.30");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=843.41,6247.06");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1847.12,6182.52");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4509.57,6236.22");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5258.87,6132.91");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=54.29,5793.15");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1080.99,5834.49");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1485.33,5981.14");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2747.07,5990.07");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3258.14,5872.34");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3739.07,5984.43");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4111.54,5912.68");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4536.56,5874.85");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5665.31,5953.92");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=498.51,5685.49");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2201.92,5608.48");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2849.71,5563.54");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4239.40,5586.45");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5160.21,5771.85");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=811.49,5444.58");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1732.36,5498.62");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3664.87,5521.02");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3991.40,5283.90");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4779.35,5519.11");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5305.39,5419.91");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=213.34,5240.24");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1445.42,5257.63");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2580.53,5126.90");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3342.37,5109.14");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4794.39,5075.30");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5579.20,5175.99");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=120.30,4868.38");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=662.50,4800.85");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2025.63,4940.55");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3804.43,4823.24");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4326.31,4948.96");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1066.81,4776.08");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1421.97,4720.09");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2558.15,4539.00");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2882.94,4701.47");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3459.32,4710.07");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4145.73,4603.79");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5215.19,4660.03");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5702.07,4636.39");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2009.05,4369.88");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3060.25,4392.49");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3565.98,4358.16");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4698.75,4309.86");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5909.50,4336.97");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=386.67,4160.35");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=745.51,4110.73");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1111.17,4171.07");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1509.55,4124.51");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5093.35,4098.29");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5540.98,4285.05");
    }

}

