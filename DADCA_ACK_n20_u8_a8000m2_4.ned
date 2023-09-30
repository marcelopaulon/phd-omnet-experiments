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

network DADCA_ACK_n20_u8_a8000m2_4
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
        @display("i=misc/sensor;p=2054.55,7950.24");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2890.18,7959.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3280.16,7911.79");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4755.77,7996.79");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5223.34,7839.80");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5947.68,7839.14");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=468.00,7636.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1036.43,7627.69");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1392.03,7725.30");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2440.94,7521.84");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2942.07,7560.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4205.34,7620.67");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4807.47,7610.45");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=114.96,7504.45");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=856.47,7258.47");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1419.96,7325.18");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2095.82,7320.85");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3552.17,7439.00");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4647.68,7298.02");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5600.20,7369.83");
    }

}

