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

network DADCA_ACK_n100_a8000m2_8
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
        @display("i=misc/sensor;p=222.50,7973.08");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=633.66,7877.46");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1424.74,7975.64");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2019.21,7923.48");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2771.59,7762.80");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3369.55,7910.67");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=934.19,7675.81");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1524.24,7602.42");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2146.00,7549.74");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3801.49,7628.12");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4365.69,7561.73");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5160.97,7577.23");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=368.07,7320.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3153.13,7393.09");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5695.81,7488.09");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=935.65,7180.30");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1727.57,7150.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2561.15,7162.78");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3614.01,7252.57");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4121.48,7024.37");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4687.28,7079.18");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5124.78,7123.51");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1995.63,6830.34");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2922.28,6884.55");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3545.52,6830.53");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5761.15,6984.69");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=191.60,6723.75");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=635.02,6703.54");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1008.71,6529.63");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1451.07,6635.76");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2583.35,6626.27");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3889.00,6574.74");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4384.33,6742.53");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5142.75,6643.87");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5496.97,6738.30");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5804.79,6539.57");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=81.04,6377.45");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1667.84,6303.86");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3036.51,6325.29");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5379.80,6363.32");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=544.82,6034.36");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1266.31,6071.40");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2010.05,6110.27");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2478.35,6158.60");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3553.38,6242.55");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4117.68,6207.55");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4564.06,6116.98");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4929.79,6064.30");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5706.00,6141.85");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1002.75,5800.12");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1631.13,5799.68");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3094.59,5970.33");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3446.79,5783.57");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5377.92,5920.74");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=124.76,5597.46");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2339.29,5733.80");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2788.81,5746.86");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4765.57,5598.12");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5737.82,5768.16");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=716.81,5317.22");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2546.06,5376.46");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3054.97,5389.63");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3868.66,5453.23");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4367.70,5447.81");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5105.09,5505.62");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5646.63,5394.34");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1415.02,5193.30");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2029.97,5259.06");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3238.11,5048.52");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3669.16,5159.84");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4997.14,5167.79");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=180.43,5015.26");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1149.17,4825.73");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1535.95,4857.57");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=2195.27,4801.30");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2636.71,4825.38");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=3648.74,4808.94");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4552.54,5006.68");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5620.63,4859.73");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=227.59,4535.74");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=607.03,4682.89");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3014.35,4587.91");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4067.67,4751.86");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=4741.75,4710.21");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5125.63,4652.67");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=909.43,4328.46");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1992.15,4365.65");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2767.74,4298.94");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4433.33,4369.42");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1671.10,4175.44");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2310.47,4206.86");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3269.04,4138.41");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3759.28,4254.97");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5169.47,4183.54");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5581.68,4136.63");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=5961.58,4274.78");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=59.62,3921.46");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=448.55,3918.64");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=985.86,3922.26");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1354.22,3996.30");
    }

}

