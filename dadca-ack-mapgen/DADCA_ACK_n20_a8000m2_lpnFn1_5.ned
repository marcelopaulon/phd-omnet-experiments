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

network DADCA_ACK_n20_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=58.91,169.10");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=144.88,211.62");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=227.83,160.46");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=316.27,162.71");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=254.64,79.08");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=181.95,110.70");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=345.71,42.70");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=416.32,55.02");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=430.37,214.27");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=439.24,306.86");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=353.15,289.97");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=362.88,394.13");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=348.93,457.99");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=390.31,523.18");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=426.54,371.98");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=511.84,219.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=302.34,330.58");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=221.79,299.21");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=412.18,152.50");
    }

}

