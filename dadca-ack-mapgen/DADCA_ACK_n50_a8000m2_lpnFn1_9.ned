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

network DADCA_ACK_n50_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor,p=176.95,58.08");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=20.46,27.40");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=57.38,200.37");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=156.26,208.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=167.67,128.52");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=105.97,11.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=49.30,292.81");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=107.36,327.60");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=175.52,270.48");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=229.05,241.55");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=308.97,296.64");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=242.29,356.95");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=132.89,398.08");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=39.97,427.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=28.43,360.45");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=156.56,477.73");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=180.80,344.56");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=379.47,305.58");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=251.65,133.80");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=248.36,56.65");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=309.82,17.01");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=323.15,235.02");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=478.07,297.69");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=433.44,195.31");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=414.04,115.91");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=541.87,236.29");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=488.19,155.84");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=657.86,244.18");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=719.46,180.74");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=661.00,128.42");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=592.12,85.65");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=518.30,85.17");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=634.92,27.61");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=718.02,13.23");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=552.36,167.10");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=583.15,304.72");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=572.30,393.07");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=538.10,486.50");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=621.19,468.70");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=573.78,536.88");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=542.14,616.14");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=477.88,19.08");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=389.05,26.90");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=367.99,178.86");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=473.32,363.09");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=696.16,334.40");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=751.44,367.59");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=814.59,343.81");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=807.44,418.75");
    }

}

