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

network DADCA_ACK_n20_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=34.44,62.22");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=60.80,148.18");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=155.48,143.10");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=116.47,190.28");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=241.88,200.71");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=269.94,261.53");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=336.61,329.26");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=376.66,268.23");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=401.89,333.71");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=440.79,272.56");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=368.26,208.31");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=417.10,147.29");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=322.04,143.26");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=322.57,51.14");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=261.96,40.23");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=382.28,38.44");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=451.10,16.86");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=552.31,0.57");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=604.59,59.00");
    }

}

