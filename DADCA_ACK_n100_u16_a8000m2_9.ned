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

network DADCA_ACK_n100_u16_a8000m2_9
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
        @display("i=misc/sensor;p=132.41,7892.61");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=807.10,7994.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1557.27,7761.26");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1986.53,7847.70");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2806.03,7980.06");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3637.34,7960.12");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4319.40,7846.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1166.74,7519.96");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2596.85,7684.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3075.82,7552.82");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3888.29,7636.59");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5038.05,7727.23");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5480.50,7553.17");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5924.87,7660.46");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=239.82,7324.83");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=634.23,7473.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1797.02,7404.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3488.43,7278.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4513.02,7270.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4869.61,7291.17");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=951.98,7136.72");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1483.44,7218.08");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1965.21,7081.05");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2439.73,7231.88");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3812.36,7026.55");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5806.48,7190.72");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=394.51,6980.78");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2781.99,6852.86");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3150.36,6985.91");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3525.38,6804.58");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4165.03,6966.87");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4776.06,6821.44");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5373.91,6809.00");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1445.76,6758.56");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2264.88,6556.43");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3116.70,6544.60");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4254.92,6589.96");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5683.65,6641.01");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=151.47,6302.61");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=928.31,6494.13");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1809.85,6307.16");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2735.41,6327.78");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3698.48,6421.07");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4632.56,6460.79");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5845.63,6325.15");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=552.60,6101.75");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1111.77,6145.40");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1463.22,6193.20");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2396.35,6144.85");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3139.98,6077.25");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3674.10,6053.96");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4004.63,6201.47");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4614.24,6067.70");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5224.36,6209.78");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1947.09,5838.59");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5655.73,5784.29");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=266.56,5566.04");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=853.99,5609.64");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1429.69,5751.50");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2777.16,5722.10");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3578.25,5695.53");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4826.25,5527.40");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5227.22,5715.41");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2278.65,5289.17");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3462.76,5348.34");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4077.57,5495.95");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5212.30,5282.92");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5758.70,5284.25");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=127.10,5138.91");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=728.12,5164.11");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1432.74,5219.58");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2918.89,5135.72");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4388.28,5043.33");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4881.37,5123.38");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1996.81,4909.78");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3549.93,4851.27");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5591.77,4947.06");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=501.73,4758.63");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1064.23,4559.10");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1561.58,4755.15");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2952.56,4627.32");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3965.67,4691.91");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4375.88,4555.86");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5792.05,4657.22");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=19.15,4299.54");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=413.08,4407.88");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1665.63,4332.95");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3286.78,4498.56");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4768.73,4506.40");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5192.52,4440.27");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1238.20,4120.80");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2116.36,4132.43");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2697.35,4202.06");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3222.58,4058.62");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3580.85,4188.16");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4470.98,4208.84");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5117.33,4076.80");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5475.31,4118.26");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5995.65,4232.30");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=799.81,4016.78");
    }

}

