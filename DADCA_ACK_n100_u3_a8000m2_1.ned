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

network DADCA_ACK_n100_u3_a8000m2_1
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
        @display("i=misc/sensor;p=492.94,7828.36");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1392.44,7953.54");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2489.75,7947.89");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3446.12,7896.95");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5684.95,7852.97");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1012.08,7618.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1808.19,7521.74");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2973.82,7740.15");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3789.96,7623.66");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4479.45,7712.22");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5060.47,7654.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=623.57,7384.87");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2187.62,7484.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3284.10,7319.91");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4609.03,7350.94");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4984.41,7283.21");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5682.46,7421.97");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2.09,7133.33");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1975.98,7054.54");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2698.39,7137.89");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5239.53,7025.14");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=532.85,6858.02");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=973.12,6866.50");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3254.96,6835.52");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4497.86,6959.75");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5828.08,6776.63");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1408.87,6710.70");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2384.46,6534.76");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3785.61,6520.99");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5081.53,6639.29");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5495.37,6532.07");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=62.13,6342.60");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=436.98,6490.73");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=794.46,6359.64");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3028.92,6348.17");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4518.37,6473.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5836.73,6337.05");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1151.85,6233.07");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1903.47,6260.59");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2627.71,6202.62");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4138.31,6209.11");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5086.13,6232.33");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=161.50,5941.49");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=514.06,5875.60");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=894.95,5970.21");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1450.62,5876.99");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2096.65,5923.61");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2958.19,5881.53");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3529.23,5982.21");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4476.05,5860.92");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5754.04,5989.22");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2554.37,5726.69");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3338.15,5649.31");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=3904.81,5583.93");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5283.90,5705.62");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5894.14,5607.13");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=385.86,5518.74");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1047.50,5428.87");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1660.44,5375.21");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2347.60,5438.28");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2717.81,5281.60");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4224.14,5372.16");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4979.52,5508.94");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=107.82,5127.60");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=662.69,5216.87");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1247.84,5041.04");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3144.67,5277.58");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3921.66,5163.80");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5091.59,5151.29");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5538.81,5232.20");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=888.18,4853.08");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2018.64,4980.92");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2487.21,4840.96");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3479.16,4986.43");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4118.89,4787.89");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4787.15,4968.53");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=184.30,4707.06");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1598.57,4687.64");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2043.38,4595.48");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3106.30,4736.71");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4621.24,4639.99");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5077.73,4611.06");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5557.87,4686.42");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=413.72,4425.16");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1085.06,4322.04");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2389.10,4407.84");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2893.80,4385.53");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3486.91,4514.70");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3897.43,4479.56");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5943.30,4380.87");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=709.51,4184.66");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1716.18,4206.72");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4436.75,4141.80");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=4901.50,4096.03");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5565.83,4075.16");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=404.23,3949.59");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1320.33,3943.08");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3105.90,3938.84");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3994.04,4028.66");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=5994.21,3891.83");
    }

}

