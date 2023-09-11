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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=160.75,92.91");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=160.65,163.77");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=242.83,117.06");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=291.71,158.23");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=200.32,8.26");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=54.57,59.97");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=117.20,2.97");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=8.17,184.29");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=80.18,294.79");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=144.56,249.76");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=98.63,175.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=14.69,5.64");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=267.12,262.02");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=195.42,283.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=145.52,351.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=5.44,318.09");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=52.26,420.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=111.15,483.06");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=262.84,377.65");
    }

}

