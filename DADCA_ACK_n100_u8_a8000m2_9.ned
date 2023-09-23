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
        @display("i=misc/sensor;p=962.65,7906.81");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1867.14,7934.83");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2940.12,7850.56");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3990.89,7805.01");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4369.47,7821.37");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=839.95,7515.94");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1249.05,7672.62");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1786.57,7590.86");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2356.45,7668.19");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3387.84,7633.73");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4947.02,7633.37");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5786.02,7577.28");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=367.95,7263.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2106.85,7314.08");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2711.13,7485.85");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3961.67,7276.57");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4519.97,7400.60");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5490.04,7326.66");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=835.44,7092.37");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1524.42,7103.29");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3054.67,7218.67");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3550.69,7213.98");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2117.35,6881.10");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2651.51,7002.63");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3293.57,6871.71");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4185.02,6914.10");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4566.08,6841.86");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5060.55,7003.38");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5684.39,6924.96");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=751.87,6713.35");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1158.76,6532.37");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1594.57,6667.19");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3862.27,6635.42");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5182.46,6651.29");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=463.47,6458.13");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2114.95,6313.62");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2724.31,6390.13");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3092.30,6353.66");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4535.23,6335.85");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5756.24,6343.78");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=43.01,6238.63");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=999.40,6063.81");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1432.33,6172.83");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3516.61,6243.84");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4200.63,6200.68");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5064.42,6198.38");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=342.70,6012.25");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1923.58,5784.07");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2541.83,5930.22");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2903.87,5980.96");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4011.89,5867.85");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4428.54,5919.72");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5692.52,5858.34");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1272.11,5695.09");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2440.28,5533.85");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3035.13,5560.67");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3580.81,5761.58");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5315.98,5552.82");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=330.85,5486.00");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=722.47,5467.89");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1831.60,5391.87");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2719.60,5302.21");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3424.67,5305.13");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4732.67,5523.38");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5762.53,5454.26");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1043.47,5045.45");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1483.62,5078.38");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2335.69,5168.43");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3762.76,5153.10");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=4190.41,5220.57");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5095.82,5251.41");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=61.41,4840.28");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=691.61,4882.55");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3230.11,4998.31");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4690.11,4928.11");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5269.58,4914.75");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5906.98,5021.45");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=314.62,4595.10");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1733.44,4749.14");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2180.35,4558.54");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2551.48,4567.51");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2916.92,4664.33");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3532.04,4682.73");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4162.54,4720.55");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4503.85,4563.23");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=628.45,4364.25");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=991.22,4380.52");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1371.07,4366.23");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3240.74,4390.09");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5102.62,4501.93");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5666.64,4479.63");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=279.60,4055.22");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1760.58,4169.65");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2654.51,4140.06");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3436.78,4090.17");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4244.92,4168.56");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4822.95,4260.01");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=916.61,3887.16");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1400.35,3862.83");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2167.40,3981.43");
    }

}

