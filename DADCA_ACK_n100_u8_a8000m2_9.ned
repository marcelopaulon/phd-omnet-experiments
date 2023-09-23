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

network DADCA_ACK_n100_u8_a8000m2_9
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
        @display("i=misc/sensor;p=355.54,7909.88");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=874.22,7999.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1681.63,7948.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2308.75,7803.90");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2894.45,7969.95");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3349.37,7874.28");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4196.11,7807.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5234.59,7870.29");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5803.13,7772.95");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=157.27,7538.02");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1138.79,7715.82");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1967.26,7545.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3119.71,7513.35");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3670.70,7615.42");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5503.89,7586.63");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=745.38,7363.44");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2692.74,7467.41");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3422.18,7320.22");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4213.31,7302.13");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4701.21,7494.92");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=498.22,7011.76");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1142.89,7100.41");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1520.40,7149.78");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2039.51,7154.59");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2921.75,7046.93");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3805.45,7159.78");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5114.37,7068.57");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5891.76,7073.44");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=75.00,7001.54");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2301.17,6857.97");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4093.59,6823.02");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4538.53,6954.37");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5568.18,6836.22");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=319.56,6535.44");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=768.98,6640.41");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1132.20,6736.53");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1442.47,6532.58");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1767.79,6677.32");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2899.59,6644.57");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3509.76,6516.37");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4884.92,6523.84");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5892.88,6530.56");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1058.51,6359.13");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2307.60,6409.66");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2710.65,6325.71");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4307.20,6364.60");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=679.79,6249.69");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2042.71,6171.07");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4822.27,6095.09");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5301.49,6190.41");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5659.66,6182.73");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=433.33,5941.28");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=957.32,5939.68");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1724.02,5935.17");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2330.19,5784.59");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3094.88,6013.83");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3875.67,6013.31");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1326.12,5714.65");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2679.21,5640.60");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3607.15,5561.84");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4354.59,5747.55");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4818.30,5675.46");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5564.77,5664.88");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=487.04,5487.05");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1604.29,5403.15");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2072.17,5503.77");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3167.81,5308.33");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4023.54,5500.01");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=691.99,5051.90");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1095.49,5143.27");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1869.00,5174.00");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2234.11,5179.19");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2601.79,5262.25");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3997.07,5045.13");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4500.04,5182.26");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5386.63,5144.72");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5804.43,5212.62");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1451.91,4801.45");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2420.59,4799.90");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3102.19,4918.27");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3451.69,4996.59");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4908.75,5020.79");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2115.38,4604.84");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5781.99,4646.95");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=447.30,4401.36");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1919.51,4301.53");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2587.99,4400.85");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2962.15,4507.99");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3773.49,4399.47");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4246.34,4463.10");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4649.71,4458.93");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5301.64,4479.24");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1043.49,4219.34");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1567.08,4206.42");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2923.17,4070.25");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3380.01,4174.95");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5204.68,4081.89");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5788.02,4070.03");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1633.60,3838.08");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2575.08,4019.95");
    }

}

