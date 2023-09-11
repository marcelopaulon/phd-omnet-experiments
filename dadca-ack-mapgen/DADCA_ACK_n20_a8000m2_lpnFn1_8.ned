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

network DADCA_ACK_n20_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=33.09,173.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=101.26,183.75");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=39.16,234.39");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1.96,294.87");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=48.69,377.45");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=15.04,468.71");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=111.00,495.12");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=113.86,403.53");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=178.85,478.46");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=215.29,430.23");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=254.99,338.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=281.69,410.05");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=377.04,424.92");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=343.90,365.41");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=175.89,316.44");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=158.67,248.17");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=66.13,555.29");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=39.38,651.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=115.97,686.94");
    }

}

