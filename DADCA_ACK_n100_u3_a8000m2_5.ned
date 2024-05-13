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

network DADCA_ACK_n100_u3_a8000m2_5
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
        @display("i=misc/sensor;p=2023.60,7941.95");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2722.43,7828.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3312.65,7952.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4349.63,7815.95");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5324.80,7921.33");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=176.81,7716.60");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=602.78,7716.76");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1180.46,7519.69");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5180.42,7562.82");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5821.46,7739.99");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1781.52,7448.42");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2477.66,7435.40");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2984.49,7430.57");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3566.04,7501.67");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=172.16,7225.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=641.83,7209.04");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1454.03,7082.01");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2124.38,7165.83");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2520.25,7049.86");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3371.34,7047.08");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4088.82,7193.26");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5362.17,7130.23");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5744.33,7206.98");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=972.25,6943.12");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3023.10,6794.73");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3706.67,6797.00");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4712.33,6991.20");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=242.07,6725.52");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=681.77,6571.58");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1158.17,6591.38");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1730.51,6715.24");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4197.58,6603.23");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4941.26,6702.76");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5482.16,6716.32");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5884.32,6680.14");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=106.12,6342.18");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2215.66,6447.73");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2705.53,6389.63");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3114.39,6398.56");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3503.04,6419.00");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4680.28,6445.59");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5354.10,6376.02");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=564.90,6170.08");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1084.24,6058.04");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1499.06,6232.11");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1815.00,6060.01");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3212.66,6059.06");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4366.82,6199.00");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5956.41,6140.30");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2158.23,5959.35");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2754.69,5916.99");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3618.26,5942.16");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4233.78,5853.73");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4888.35,5945.58");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5245.77,6013.87");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5605.62,6018.35");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5897.26,5793.66");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=207.76,5669.92");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=967.51,5641.91");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1507.95,5632.10");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3254.64,5645.85");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4548.42,5635.48");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5427.50,5677.78");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2135.97,5463.57");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2743.91,5471.68");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3566.62,5345.28");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4271.91,5407.70");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5139.95,5399.72");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=688.84,5263.75");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1606.16,5208.84");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2261.73,5034.66");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3051.61,5185.99");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4565.61,5179.96");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5580.69,5165.59");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5998.47,5254.11");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=280.72,4955.54");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=917.57,4980.05");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1309.98,4960.96");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1904.59,4817.80");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3960.52,4945.10");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5083.61,4867.92");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5758.70,4790.22");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2327.61,4672.76");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2711.26,4754.02");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3312.16,4778.12");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3698.95,4616.88");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4409.70,4586.40");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=476.08,4349.25");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1042.16,4461.68");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1556.14,4424.22");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1995.54,4409.95");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2971.78,4472.13");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4067.13,4455.36");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4750.02,4472.00");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5172.57,4298.64");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5823.74,4300.64");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1269.79,4185.87");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1771.87,4120.55");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2527.44,4043.79");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3161.37,4176.51");
    }

}

