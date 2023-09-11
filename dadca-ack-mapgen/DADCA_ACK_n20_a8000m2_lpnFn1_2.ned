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

network DADCA_ACK_n20_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=156.62,138.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=137.61,220.67");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=197.64,40.51");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=245.66,89.32");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=132.15,22.52");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=260.54,211.13");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=282.82,143.24");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=355.62,86.90");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=417.53,43.85");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=205.08,269.29");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=366.33,177.08");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=111.33,321.89");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=147.80,371.27");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=149.31,447.56");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=56.21,480.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=23.62,413.04");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=115.50,539.84");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=137.29,633.12");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=168.96,685.83");
    }

}

