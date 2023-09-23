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

network DADCA_ACK_n100_u4_a8000m2_6
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
        @display("i=misc/sensor;p=475.99,7772.97");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=952.51,7963.46");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1403.38,7894.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2736.67,7753.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3137.94,7786.60");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3946.73,7926.64");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4460.74,7763.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4806.82,7903.16");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5780.82,7977.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2293.50,7573.02");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3686.46,7528.83");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4068.28,7520.37");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5013.26,7579.15");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=836.18,7448.41");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1309.84,7482.64");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1862.48,7363.99");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2953.22,7348.39");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5388.64,7400.99");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5920.96,7257.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=320.46,7166.10");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1061.38,7137.59");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2204.44,7021.27");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2546.64,7133.00");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3266.82,7140.54");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3638.52,7087.90");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4161.67,7010.79");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4692.83,7232.38");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=745.67,6941.26");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1611.15,6766.03");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4431.13,6787.07");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5035.70,6927.85");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5645.48,7009.74");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=153.97,6578.57");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=657.61,6576.04");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1178.03,6614.66");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2408.21,6676.59");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3079.83,6695.61");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3651.04,6660.26");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5585.01,6624.74");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1934.34,6504.58");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3352.96,6434.66");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3885.44,6373.75");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4290.97,6300.82");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4670.91,6469.98");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5152.61,6432.45");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=499.14,6229.16");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=937.56,6262.59");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1440.26,6227.62");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1787.71,6032.36");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2303.26,6228.57");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2664.43,6070.37");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3178.61,6034.94");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4655.13,6028.26");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5151.41,6024.05");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5766.91,6179.89");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=36.75,5936.20");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=714.30,5824.20");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1209.71,5844.39");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3580.03,5831.59");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1911.82,5583.93");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2340.35,5715.41");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2931.95,5526.18");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4309.53,5714.42");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4854.92,5595.78");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5981.83,5600.66");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=498.26,5513.11");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1619.75,5357.98");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2560.55,5406.38");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3314.57,5426.60");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3790.08,5305.69");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5639.51,5477.68");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=378.40,5143.02");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=991.22,5235.96");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2159.93,5263.31");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2734.08,5065.43");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4388.26,5123.91");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5004.09,5153.90");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5470.78,5146.67");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1631.38,5004.50");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2136.87,4794.30");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3158.43,4898.59");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3751.37,4911.57");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4152.69,4801.96");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5795.01,4992.55");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=426.35,4767.85");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1229.86,4779.07");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1761.36,4580.89");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2494.26,4660.88");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3454.83,4638.44");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4785.18,4612.63");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5245.21,4681.18");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5575.25,4556.87");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=677.03,4293.26");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=995.99,4481.03");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=3103.78,4531.49");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3930.08,4514.15");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4531.65,4354.98");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1000.50,4045.90");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1408.04,4140.64");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1929.49,4232.17");
    }

}

