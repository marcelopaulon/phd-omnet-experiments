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

network DADCA_ACK_n100_u8_a8000m2_1
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
        @display("i=misc/sensor;p=181.58,7852.64");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=547.73,7964.52");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1834.40,7805.31");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2460.96,7899.16");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4078.11,7776.91");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4734.67,7762.70");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2314.97,7509.95");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2909.68,7722.11");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3387.94,7732.81");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=435.32,7495.76");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=856.27,7480.76");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1412.27,7487.58");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1853.64,7299.12");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2973.80,7337.87");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4361.94,7385.64");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4733.64,7382.11");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1086.57,7081.43");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2598.96,7214.00");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3477.50,7112.54");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3905.88,7207.13");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4820.00,7031.33");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5344.49,7117.60");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=800.95,6859.01");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1928.14,6843.84");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2346.87,6769.32");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2710.99,6793.51");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3111.30,6821.50");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4217.43,7000.20");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=179.35,6726.39");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3479.17,6688.21");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3927.33,6524.63");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4315.01,6604.75");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4874.99,6526.88");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5699.39,6719.89");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=632.63,6494.31");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1073.40,6318.62");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2859.34,6367.53");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=81.46,6227.70");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1824.07,6168.73");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2433.43,6201.53");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3266.23,6244.66");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4438.18,6128.78");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4829.02,6142.22");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5378.63,6103.15");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5763.40,6222.91");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=88.71,5858.54");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=538.30,5864.10");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1019.72,5950.52");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1541.10,5912.56");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2096.47,5921.88");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2794.54,5968.74");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3183.81,5862.48");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3839.82,5928.98");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1272.86,5637.43");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2563.10,5564.74");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3579.07,5532.69");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4834.78,5741.44");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5531.15,5593.11");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5890.75,5630.04");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=894.73,5447.45");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1507.47,5318.77");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1929.80,5456.27");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4337.10,5434.13");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=309.51,5170.18");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2315.82,5108.26");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3131.53,5247.52");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5560.20,5226.31");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5996.67,5254.91");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=947.08,4920.06");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1680.14,4978.93");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2022.21,4867.50");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4748.86,5029.38");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5832.01,4843.80");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=607.92,4690.69");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2458.37,4581.99");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2865.26,4715.11");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3401.25,4776.99");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3992.46,4750.88");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4383.46,4778.48");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5328.42,4697.56");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=161.17,4368.27");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1106.16,4315.03");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1519.95,4446.69");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1937.13,4382.56");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3395.25,4380.21");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3889.46,4358.62");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4838.67,4401.67");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=664.76,4193.52");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1654.64,4076.86");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2505.65,4161.99");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4282.11,4160.95");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5161.31,4041.14");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5821.70,4201.62");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=271.96,3869.41");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1293.84,3975.68");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1881.90,3794.98");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3144.06,3987.45");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4062.08,3826.00");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=4683.98,3825.25");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5563.81,3891.58");
    }

}

