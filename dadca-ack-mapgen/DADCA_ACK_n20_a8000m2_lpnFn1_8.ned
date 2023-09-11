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
        @display("i=misc/sensor;p=99.93,184.23");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=188.15,210.92");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=237.36,282.00");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=183.71,343.17");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=121.14,299.23");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=70.01,361.66");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=37.78,278.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=163.37,408.90");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=97.43,478.79");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=47.76,434.34");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=230.94,415.04");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=185.08,491.88");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=236.43,533.88");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=319.04,533.49");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=285.21,596.35");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=238.04,644.49");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=329.74,676.84");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=397.43,722.42");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=357.10,784.49");
    }

}

