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

network DADCA_ACK_n50_u3_a8000m2_3
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
        @display("i=misc/sensor;p=264.55,7835.89");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=812.83,7972.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2450.14,7952.47");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3434.83,7951.99");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3883.96,7921.68");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4608.01,7871.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=760.57,7545.33");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1308.01,7706.72");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2745.17,7731.86");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3164.24,7695.86");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4388.05,7577.09");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4985.96,7632.47");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5703.63,7660.36");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=218.42,7313.95");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1762.79,7422.44");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2203.07,7500.88");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3387.23,7324.48");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3754.05,7484.05");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=798.46,7195.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1200.69,7085.88");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1783.44,7052.05");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2711.82,7250.98");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3950.23,7064.78");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4587.44,7105.61");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=414.70,7002.06");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2196.11,6848.39");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3328.84,6964.09");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4251.64,6791.64");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5095.02,6974.07");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5694.64,6978.44");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=180.76,6519.87");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=922.33,6713.03");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1504.47,6655.13");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2623.30,6760.25");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3722.06,6669.85");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4672.28,6744.67");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5542.83,6527.24");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1169.54,6393.61");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2001.44,6470.10");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2897.95,6356.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3308.16,6409.68");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3895.63,6302.70");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4324.46,6435.95");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4905.65,6380.83");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=26.27,6030.20");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=562.03,6145.77");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1503.18,6217.20");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2008.96,6115.84");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2444.27,6255.54");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4627.88,6021.41");
    }

}

