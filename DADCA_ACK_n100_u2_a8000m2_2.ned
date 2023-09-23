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

network DADCA_ACK_n100_u2_a8000m2_2
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
        @display("i=misc/sensor;p=918.86,7969.91");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1426.41,7970.74");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1937.68,7978.29");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3269.38,7757.37");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4446.90,7932.64");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5128.31,7848.92");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5596.99,7934.71");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=401.78,7691.65");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=993.29,7531.87");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1461.73,7520.70");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1815.29,7639.53");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2319.10,7741.53");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3795.19,7657.80");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2660.25,7426.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2988.43,7285.95");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4384.39,7377.58");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4851.77,7374.18");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5530.88,7350.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=60.38,7167.44");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1177.67,7030.82");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1695.14,7170.76");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2270.52,7180.64");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3397.28,7094.50");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4048.95,7098.87");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=86.00,6799.10");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=623.62,6941.74");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1765.90,6766.46");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2274.85,6773.73");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2797.14,6950.34");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4704.71,6955.65");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5121.46,6929.38");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5619.24,6987.26");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=423.91,6552.11");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=929.07,6584.00");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1352.19,6656.73");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3179.60,6596.79");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4562.93,6633.78");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5447.36,6634.92");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5978.28,6515.58");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=81.01,6283.23");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2601.74,6451.33");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3554.05,6390.22");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4035.10,6313.98");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4928.63,6312.12");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5660.27,6354.68");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=875.74,6169.01");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1376.96,6241.12");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1755.79,6133.81");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2194.33,6161.49");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2861.22,6162.39");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3522.03,6025.08");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=4650.70,6069.91");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=299.91,5991.25");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2439.02,5794.64");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3147.49,5948.38");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3986.56,5790.33");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5510.26,5999.32");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=865.72,5614.30");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1376.34,5579.88");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4398.05,5718.13");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5117.35,5709.54");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5781.18,5619.56");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=275.67,5344.63");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=630.33,5342.71");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=1957.31,5315.13");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=2982.14,5385.91");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3535.69,5298.29");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3967.90,5318.84");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4658.63,5288.59");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1133.00,5265.85");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2405.30,5246.18");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=5027.97,5244.09");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5442.06,5273.04");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5794.12,5172.19");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=384.24,5010.66");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1709.74,4846.10");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2234.13,4826.49");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2664.65,4797.38");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3108.51,5029.19");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4265.13,4982.00");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5524.64,4846.19");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=209.80,4656.16");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1046.52,4752.68");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3504.23,4662.91");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3863.97,4773.17");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5063.50,4730.38");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5835.07,4635.38");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1336.54,4484.46");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2190.73,4428.99");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2925.39,4529.03");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3938.97,4293.41");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4761.29,4512.82");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5267.97,4336.77");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=249.12,4099.37");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=982.09,4075.26");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1649.20,4206.63");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2571.15,4232.43");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4424.42,4283.98");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1942.55,3947.62");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2453.21,3868.92");
    }

}

