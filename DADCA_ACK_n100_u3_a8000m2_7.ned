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

network DADCA_ACK_n100_u3_a8000m2_7
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
        @display("i=misc/sensor;p=508.81,7762.26");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=878.19,7887.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1398.47,7956.65");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2233.83,7945.86");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2723.94,7966.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3994.63,7857.36");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4507.15,7962.52");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5386.41,7805.05");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5815.21,7891.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=998.85,7558.44");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3340.73,7697.18");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=256.44,7274.06");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1482.62,7451.44");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2280.50,7429.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2673.82,7468.01");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3111.12,7361.76");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3886.11,7417.32");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4402.32,7276.67");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4946.15,7499.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5283.40,7275.67");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5778.90,7457.23");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1771.33,7203.68");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2603.74,7105.50");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3543.89,7030.89");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4782.79,7106.99");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=243.22,6805.21");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=720.75,6928.56");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1097.62,6965.94");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1649.04,6807.95");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2222.78,6946.70");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3061.67,7010.01");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4434.96,6775.24");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5878.71,6960.53");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2567.54,6752.06");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3440.09,6624.31");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3969.06,6758.05");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=160.67,6302.71");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=727.44,6334.67");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1135.75,6347.52");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1574.45,6443.87");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2169.77,6332.33");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3079.90,6433.11");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5087.39,6476.06");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5923.45,6443.50");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1887.28,6035.59");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2546.18,6258.08");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3005.27,6068.17");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3441.45,6094.92");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3931.41,6077.70");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4495.65,6174.16");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=804.63,5865.01");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1348.08,5935.18");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4961.56,5950.26");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5582.67,5976.13");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=153.77,5624.52");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1655.21,5567.04");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2148.75,5721.48");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2683.27,5647.28");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4331.22,5660.52");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=800.19,5407.25");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1319.56,5286.18");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3263.43,5414.19");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3827.88,5409.29");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4602.69,5422.55");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5216.45,5489.75");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5771.16,5501.52");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=544.34,5040.53");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1889.97,5209.21");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4152.47,5271.85");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=177.87,4925.44");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1149.09,4906.41");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1560.57,4973.76");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2427.35,4975.73");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3045.72,4955.67");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3803.77,4992.20");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4295.32,4912.00");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5125.56,4960.80");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=576.86,4605.46");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1225.88,4544.86");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1649.44,4584.92");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2034.30,4567.92");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3134.11,4554.94");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3501.77,4612.29");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4197.88,4540.86");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4776.40,4728.56");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=27.31,4455.99");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2616.64,4469.92");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3808.03,4310.88");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5258.37,4489.67");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=373.90,4285.82");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1046.50,4140.23");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2033.11,4106.49");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2715.75,4068.67");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3269.55,4071.00");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4687.02,4165.25");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5126.69,4100.89");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=139.01,4026.12");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=594.51,3943.27");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1691.58,3970.82");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2373.43,3876.65");
    }

}

