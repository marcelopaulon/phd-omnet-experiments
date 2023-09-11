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

network DADCA_ACK_n50_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=144.76,158.86");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=162.81,246.97");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=87.06,198.47");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=87.95,294.40");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=106.88,358.33");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=112.67,447.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=5.11,352.28");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=11.27,260.43");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=37.92,29.35");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=38.91,160.78");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=217.02,374.19");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=252.76,317.21");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=307.82,280.45");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=366.60,235.99");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=412.78,357.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=478.07,400.28");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=482.14,496.92");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=378.23,488.91");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=287.17,528.13");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=373.16,553.17");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=299.34,599.90");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=351.32,675.22");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=401.22,633.00");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=454.93,693.64");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=488.75,748.81");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=550.36,785.73");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=596.33,681.60");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=620.89,746.99");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=517.42,629.65");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=267.34,653.45");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=255.19,718.04");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=196.92,617.34");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=132.18,621.49");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=68.18,664.78");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=111.31,746.86");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=169.32,805.20");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=79.90,849.78");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=145.58,865.14");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=208.36,854.76");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=275.79,816.44");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=126.46,928.67");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=163.78,981.98");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=232.31,1023.23");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=323.22,1034.68");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=352.57,1095.76");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=348.25,1171.56");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=268.19,1137.46");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=467.97,1133.76");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=458.08,1064.67");
    }

}

