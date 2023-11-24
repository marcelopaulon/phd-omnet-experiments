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

network DADCA_ACK_n100_u2_a8000m2_5
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
        @display("i=misc/sensor;p=164.19,7833.05");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=870.88,7810.61");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1662.74,7825.13");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2202.83,7977.83");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2765.94,7906.34");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3432.64,7975.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4162.79,7921.47");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5719.19,7771.96");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2511.66,7583.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3165.99,7617.37");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3835.22,7673.21");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4263.49,7584.05");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4686.00,7717.86");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5133.46,7511.96");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=628.12,7346.13");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1245.00,7363.07");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1999.24,7447.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3524.09,7402.91");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5952.29,7468.11");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=348.77,7121.41");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1824.35,7034.86");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2746.84,7217.18");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3897.36,7177.80");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4656.37,7212.44");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5523.14,7010.68");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=547.99,6833.05");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1124.51,6793.40");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2331.89,6938.71");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3135.13,6787.20");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5132.97,6979.22");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5887.76,6843.80");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=40.98,6519.96");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1513.22,6528.05");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2809.28,6523.38");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3618.16,6643.44");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4164.58,6753.45");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5027.56,6624.69");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5460.61,6518.60");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=640.21,6402.85");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2195.05,6471.92");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3422.26,6317.17");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4076.62,6294.38");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4498.70,6510.72");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1132.80,6162.52");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1661.95,6172.83");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2224.91,6076.49");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3768.45,6112.04");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4712.95,6208.93");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5063.15,6192.33");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5789.27,6108.25");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=217.20,5857.61");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2032.70,5782.19");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2574.81,5775.48");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3062.97,6014.65");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4273.58,5858.75");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4905.97,5866.17");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5408.78,6019.47");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=459.47,5573.54");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=814.44,5590.54");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2900.51,5617.02");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3469.57,5584.99");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3788.63,5759.88");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5364.02,5634.45");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1220.26,5491.37");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1723.43,5510.02");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2301.22,5494.58");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3787.82,5408.96");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=297.60,5182.83");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=792.11,5225.54");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2166.01,5124.66");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2622.02,5158.93");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3249.25,5207.92");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4098.32,5242.05");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5155.87,5264.56");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5509.27,5224.08");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1148.21,4876.41");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1729.58,4878.64");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3002.19,4924.55");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3678.26,4814.17");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4652.33,4998.20");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5866.90,4865.97");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=135.35,4604.60");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=640.88,4731.90");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1425.63,4538.58");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2178.18,4631.58");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2574.57,4702.19");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3214.10,4560.10");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5281.51,4613.59");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1009.68,4409.56");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1825.74,4503.68");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2876.13,4334.51");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3539.36,4288.38");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4324.75,4492.20");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4814.98,4400.25");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5853.86,4344.24");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=61.93,4170.01");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=617.37,4245.59");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1643.82,4199.35");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2233.80,4283.68");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2628.69,4071.93");
    }

}

