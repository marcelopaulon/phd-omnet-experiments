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

network DADCA_ACK_n100_u16_a8000m2_8
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
        @display("i=misc/sensor;p=1116.24,7990.63");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2476.69,7941.12");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3533.94,7849.78");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4163.34,7801.75");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5359.94,7945.61");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5988.09,7839.36");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=82.98,7612.72");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1554.44,7597.53");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2153.40,7625.14");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2640.04,7527.25");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3144.12,7675.24");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=472.00,7320.47");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=867.49,7490.97");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3421.50,7366.67");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3813.12,7290.24");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5267.22,7437.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5722.77,7400.08");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2023.70,7215.12");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2413.13,7255.36");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3088.67,7148.29");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4249.13,7108.89");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5095.86,7018.96");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=257.22,6976.57");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=949.42,6966.71");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2098.19,6863.54");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3508.90,6920.20");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4172.96,6763.94");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5675.49,6872.84");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=736.73,6628.16");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1836.66,6520.14");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2650.42,6746.11");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3745.25,6629.08");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4855.42,6719.91");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=503.85,6358.55");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=946.77,6305.33");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1322.14,6363.34");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2497.61,6326.92");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3349.60,6484.79");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4380.27,6361.12");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4783.37,6328.97");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5356.14,6384.66");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5737.90,6418.57");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=30.94,6138.78");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1589.41,6095.16");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2980.68,6178.68");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3500.66,6072.38");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3912.06,6261.53");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=519.77,5867.22");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2280.18,6008.37");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3109.49,5780.77");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4197.57,5942.12");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4774.89,5820.19");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5312.12,5906.59");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5703.10,5902.29");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=7.55,5752.63");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1461.05,5673.15");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2063.54,5653.22");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2653.47,5562.43");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3677.27,5745.94");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=360.57,5298.88");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1013.10,5501.56");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2993.40,5390.18");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3365.06,5491.52");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3911.99,5450.80");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4491.41,5405.05");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5114.65,5377.90");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1038.57,5104.45");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1461.18,5178.94");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2040.57,5230.56");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2504.46,5204.60");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=3666.25,5179.42");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4193.24,5152.77");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5676.27,5218.00");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=527.99,4946.97");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1670.48,4880.89");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=3123.39,4989.67");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4064.40,4823.84");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4521.62,4960.89");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4888.58,4783.42");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5926.14,4928.30");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=42.30,4720.82");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=676.00,4540.69");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1207.35,4702.38");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2011.41,4770.16");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2341.43,4571.53");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2698.02,4596.01");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=3176.39,4551.85");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3701.35,4769.44");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4366.90,4625.66");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5535.05,4781.07");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2030.93,4388.09");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4716.47,4384.65");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=385.35,4246.76");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1676.08,4279.56");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3009.22,4163.85");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3530.16,4107.00");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3870.59,4253.79");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5209.00,4258.07");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5710.05,4280.15");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=30.41,3851.66");
    }

}

