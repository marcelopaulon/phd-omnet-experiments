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

network DADCA_ACK_n100_u4_a8000m2_8
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
        @display("i=misc/sensor;p=821.37,7950.37");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1472.00,7963.66");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1963.32,7757.08");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2940.20,7991.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3557.29,7846.01");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3974.02,7880.14");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5415.86,7793.73");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=48.16,7533.43");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=431.14,7633.13");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1060.64,7682.77");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4114.32,7536.77");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4569.10,7657.48");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5012.23,7545.99");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5747.53,7669.48");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=797.13,7438.48");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1491.84,7492.17");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1962.83,7404.28");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2990.62,7482.16");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2.60,7086.16");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=412.83,7084.12");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1340.37,7166.09");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2330.44,7215.25");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3486.90,7021.98");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4530.33,7192.74");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4962.29,7057.03");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5599.75,7032.83");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=786.45,6783.06");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1531.43,6803.03");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1885.03,6948.53");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2919.85,6942.51");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5981.50,6914.97");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2272.11,6760.64");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2611.72,6598.54");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2984.42,6597.30");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3586.78,6524.40");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4191.26,6593.32");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4621.56,6584.15");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5528.86,6597.32");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=59.01,6481.04");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1649.69,6467.35");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2060.43,6362.17");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5025.97,6474.72");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5976.21,6373.35");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=488.04,6265.79");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1363.69,6213.90");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2471.14,6114.65");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2914.72,6045.52");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4184.02,6163.39");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4545.66,6155.78");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5004.33,6044.61");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5501.38,6030.91");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=198.60,5960.52");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=565.15,5829.67");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=937.96,5938.81");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2220.25,5788.37");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3489.08,5996.47");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=4085.71,5786.73");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=335.41,5550.78");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1536.66,5731.78");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3273.60,5580.65");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4602.62,5605.99");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5710.20,5704.08");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=91.37,5280.48");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=967.06,5504.74");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1308.94,5281.87");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2005.31,5344.63");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2626.77,5501.38");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3679.84,5364.93");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5453.72,5315.44");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=547.74,5241.79");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2380.44,5176.55");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4302.03,5203.20");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4756.79,5274.63");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5125.95,5036.37");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=57.22,4859.99");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1040.61,5025.19");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1874.67,4995.51");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2679.00,4950.69");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3049.12,4921.89");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3490.85,4896.49");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4020.75,4926.44");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=4797.73,4854.21");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5680.96,4951.29");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=539.27,4676.24");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1487.50,4577.71");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2232.04,4686.68");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4434.41,4770.01");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5219.04,4550.44");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=281.05,4314.01");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=924.29,4492.30");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1979.95,4308.23");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2831.13,4389.12");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3414.61,4433.87");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3854.40,4471.93");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=4590.17,4373.63");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5598.75,4497.20");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=635.49,4156.49");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2334.51,4097.39");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5100.88,4052.19");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5690.38,4091.29");
    }

}

