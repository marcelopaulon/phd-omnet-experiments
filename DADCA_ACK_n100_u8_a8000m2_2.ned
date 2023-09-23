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

network DADCA_ACK_n100_u8_a8000m2_2
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
        @display("i=misc/sensor;p=382.58,7916.72");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1959.47,7907.14");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3419.96,7774.34");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3916.52,7894.51");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4456.03,7950.29");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4796.45,7848.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5260.74,7995.76");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5799.73,7953.00");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=816.43,7693.94");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1282.56,7529.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1605.32,7666.78");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2528.04,7745.06");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=23.03,7399.11");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=780.47,7336.57");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1722.61,7323.42");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2296.78,7303.57");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3040.89,7339.27");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4151.01,7412.96");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4546.23,7374.45");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5143.76,7425.04");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5754.27,7404.10");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=335.76,7195.98");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2642.41,7169.07");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3748.16,7185.66");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4867.14,7185.93");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1195.61,6844.36");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2328.48,6836.31");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2749.36,6788.11");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3988.10,6816.99");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4403.69,6879.03");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5358.96,7002.62");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5979.83,6900.69");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=64.97,6541.38");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=545.60,6662.88");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1804.22,6744.19");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3125.76,6757.21");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4759.69,6565.71");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5819.53,6532.22");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=967.69,6385.78");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2130.21,6349.35");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2588.20,6317.01");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3218.11,6365.97");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3616.20,6514.25");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4029.10,6332.30");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4382.93,6390.57");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5358.76,6513.74");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=441.29,6201.06");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=770.51,6065.22");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1292.86,6237.69");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1692.46,6187.76");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5120.93,6032.74");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5768.83,6164.55");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=67.69,5982.83");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1035.28,5832.67");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2123.70,5933.23");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2622.79,5801.13");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3101.30,5905.22");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3679.65,5966.25");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4467.04,5858.20");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=735.28,5641.67");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1374.65,5723.71");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1807.42,5704.40");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3946.07,5569.69");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4807.42,5653.44");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=352.08,5391.53");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1350.41,5366.44");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2284.60,5407.79");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3418.48,5336.68");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4311.26,5476.44");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5274.16,5364.32");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=518.71,5058.47");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=974.62,5183.62");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1946.88,5069.40");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2559.79,5138.47");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2957.71,5100.62");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4727.03,5118.30");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5805.14,5185.46");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2371.23,4821.50");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3364.81,4891.96");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4019.60,5029.05");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4392.39,4920.36");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5427.77,4861.34");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5857.99,4807.72");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=896.00,4704.35");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1355.68,4779.56");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4410.48,4552.41");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4770.28,4683.24");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=405.19,4452.21");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1775.89,4459.84");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2737.82,4491.71");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3155.54,4482.72");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3805.75,4403.48");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5091.41,4473.20");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5921.84,4462.20");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=103.39,4194.11");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=590.54,4073.00");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1221.09,4259.79");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1613.35,4057.03");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2174.97,4281.86");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2686.46,4050.10");
    }

}

