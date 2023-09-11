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
        @display("i=misc/sensor,p=128.96,22.14");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=189.42,64.12");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=215.55,139.15");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=252.84,11.53");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=187.02,197.75");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=302.86,152.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=347.93,72.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=74.59,214.24");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=42.69,272.16");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=9.61,207.72");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=124.36,250.15");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=162.91,298.23");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=144.30,355.71");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=239.49,242.00");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=238.36,335.10");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=302.10,357.97");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=348.92,318.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=317.26,259.93");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=403.80,218.98");
    }

}

