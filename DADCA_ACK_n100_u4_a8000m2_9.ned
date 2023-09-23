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

network DADCA_ACK_n100_u4_a8000m2_9
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
        @display("i=misc/sensor;p=541.55,7774.45");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1080.48,7935.45");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2143.75,7932.11");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3246.32,7986.70");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3733.58,7904.52");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1300.54,7658.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1833.22,7613.97");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2384.60,7542.88");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3266.61,7633.89");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3945.49,7603.76");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4931.22,7574.34");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5676.56,7676.15");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=759.17,7395.08");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3610.55,7451.88");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4576.27,7438.82");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5315.18,7490.74");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5756.36,7258.25");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=231.26,7069.54");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1343.28,7191.48");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1941.86,7146.37");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2610.44,7141.07");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3163.98,7132.77");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3673.56,7033.32");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4257.71,7239.04");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5290.85,7089.61");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=617.49,7006.48");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=991.40,6897.87");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4763.94,6963.73");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5800.14,6825.77");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=512.96,6596.73");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1575.61,6526.62");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2064.79,6538.75");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2803.63,6554.89");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4116.67,6582.09");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4530.87,6689.08");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5210.92,6516.68");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=78.44,6361.33");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1021.95,6294.50");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3155.05,6277.15");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3746.88,6362.23");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4889.38,6285.70");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5936.00,6324.29");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=561.43,6026.86");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2253.56,6243.08");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4320.69,6147.22");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5506.27,6210.53");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=40.71,5942.49");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1095.68,5932.76");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1679.37,5899.15");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2710.48,5966.29");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3324.02,5793.57");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4618.43,5930.67");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5270.31,5854.44");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=287.64,5603.94");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=824.90,5647.89");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1950.49,5540.01");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2277.96,5741.63");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2894.68,5646.27");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3771.02,5725.05");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4109.64,5605.15");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4878.03,5661.94");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5975.79,5741.78");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1515.07,5432.25");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5832.39,5397.23");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=748.45,5270.47");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1168.00,5262.85");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2167.70,5057.27");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2476.44,5265.89");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3459.95,5071.03");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3982.79,5044.78");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4326.95,5170.06");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4832.04,5172.56");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5200.95,5163.34");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5906.73,5053.72");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=241.14,4792.62");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=944.21,4957.37");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1348.60,4802.29");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1780.13,4960.70");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5439.33,4864.77");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=690.90,4615.97");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2288.03,4690.32");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3067.35,4703.37");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3457.80,4690.94");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4015.68,4673.73");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4588.39,4713.16");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5088.78,4689.14");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1319.84,4443.54");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2479.54,4385.13");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3306.45,4315.89");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3803.77,4351.98");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5685.60,4397.96");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=315.09,4067.89");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1110.94,4142.99");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1784.57,4193.84");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2264.94,4064.25");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=4409.72,4192.75");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=5012.03,4240.58");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=788.33,3860.00");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1360.84,3824.36");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1987.73,3793.88");
    }

}

