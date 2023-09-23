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

network DADCA_ACK_n50_u8_a8000m2_6
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
        @display("i=misc/sensor;p=682.01,7922.39");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1258.37,7880.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2248.27,7894.46");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2839.90,7919.19");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3549.28,7851.87");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4350.77,7790.14");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4719.23,7821.21");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=49.37,7610.72");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=450.23,7559.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1673.37,7516.44");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2491.75,7637.43");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3912.00,7703.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4946.41,7550.80");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5351.97,7748.19");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5903.79,7565.13");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1080.97,7475.12");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2869.17,7258.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3575.64,7274.46");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4609.54,7418.43");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=84.15,7254.30");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=777.92,7248.49");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2273.40,7043.98");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4028.62,7214.72");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5154.76,7215.85");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5547.51,7134.62");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=212.68,6848.02");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1649.95,6829.24");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4343.45,6810.85");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=677.38,6672.81");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1203.74,6735.45");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2016.31,6600.50");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2477.64,6726.15");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3202.17,6731.57");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3812.82,6743.59");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4776.58,6622.28");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5149.98,6711.58");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5697.09,6753.67");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=385.53,6368.32");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=813.24,6339.89");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1464.23,6326.08");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2749.78,6415.39");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3265.11,6355.99");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4081.91,6348.55");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4494.57,6404.14");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5154.17,6337.70");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5566.08,6362.54");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2044.92,6051.24");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2456.51,6223.03");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4736.12,6116.75");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=199.27,5898.24");
    }

}

