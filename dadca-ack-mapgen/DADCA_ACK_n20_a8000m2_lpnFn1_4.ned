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

network DADCA_ACK_n20_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor,p=43.09,165.47");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=102.45,236.59");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=190.92,205.93");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=117.71,161.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=164.69,314.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=220.31,393.83");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=142.91,388.41");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=258.67,291.97");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=176.47,112.41");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=245.72,103.94");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=237.85,17.67");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=299.41,50.01");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=371.22,40.30");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=333.74,107.23");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=77.01,293.95");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=329.31,193.97");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=258.66,163.71");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=430.06,165.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=388.51,218.91");
    }

}

