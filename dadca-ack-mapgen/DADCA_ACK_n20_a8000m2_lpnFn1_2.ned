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
        @display("i=misc/sensor,p=36.90,81.35");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=131.03,237.21");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=187.29,178.14");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=113.75,172.64");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=24.65,202.78");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=85.28,299.22");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=15.80,285.29");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=5.93,376.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=81.56,411.37");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=137.09,463.30");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=190.13,419.98");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=290.38,462.43");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=345.48,529.68");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=299.84,607.05");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=370.92,588.45");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=441.46,540.45");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=384.95,393.39");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=374.98,323.79");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=430.51,284.01");
    }

}

