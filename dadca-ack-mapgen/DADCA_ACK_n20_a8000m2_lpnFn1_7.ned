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
        @display("i=misc/sensor,p=136.48,162.06");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=140.71,241.42");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=40.36,244.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=94.96,334.01");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=228.30,102.86");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=291.01,61.60");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=297.83,182.58");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=362.67,194.52");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=439.08,158.25");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=324.87,118.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=218.72,187.14");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=8.69,359.10");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=10.76,167.66");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=26.11,67.94");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=133.27,35.56");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=213.95,287.47");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=510.19,121.29");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=587.46,133.39");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=564.62,193.09");
    }

}

