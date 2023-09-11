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

network DADCA_ACK_n20_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=14.58,104.21");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=160.01,98.90");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=253.12,38.15");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=193.20,7.93");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=273.15,166.35");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=185.78,232.09");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=181.83,162.21");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=131.90,4.98");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=15.60,30.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=67.36,225.21");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=107.01,290.14");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=61.82,149.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=105.76,365.59");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=27.97,362.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=4.39,275.47");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=332.79,67.14");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=254.25,101.80");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=364.08,178.45");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=380.22,251.18");
    }

}

