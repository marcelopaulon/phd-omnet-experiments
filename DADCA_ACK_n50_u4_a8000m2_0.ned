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

network DADCA_ACK_n50_u4_a8000m2_0
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
        @display("i=misc/sensor;p=1161.07,7945.49");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1993.29,7900.59");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2432.22,7981.57");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3033.98,7843.33");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5558.13,7903.20");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=341.24,7695.81");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=865.70,7697.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1217.94,7576.67");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2412.49,7559.49");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3535.16,7741.25");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3959.80,7551.89");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4632.90,7569.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=159.74,7372.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=779.03,7344.40");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1826.39,7490.53");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3464.27,7332.03");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5139.40,7383.60");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5642.44,7372.94");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1241.64,7213.48");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2874.38,7145.29");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4560.58,7043.35");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=433.19,6863.87");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=806.15,6820.43");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1648.54,6894.53");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2203.22,6961.38");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3013.12,6796.64");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3801.28,6887.70");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5134.37,6985.08");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5924.72,6926.80");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=106.37,6656.79");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1357.47,6694.12");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2462.85,6667.62");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3347.41,6638.34");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3797.90,6522.49");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5094.64,6580.44");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5669.81,6553.05");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=435.03,6334.41");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=973.77,6384.70");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1775.95,6462.20");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2198.14,6394.54");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4522.72,6430.19");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1248.16,6148.01");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2638.29,6258.89");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3318.49,6232.82");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4739.68,6144.76");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5528.73,6044.47");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5987.35,6167.76");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=974.72,5905.24");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1676.20,5956.89");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2268.52,5822.94");
    }

}

