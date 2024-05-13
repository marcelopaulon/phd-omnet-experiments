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

network DADCA_ACK_n50_u2_a8000m2_7
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
        @display("i=misc/sensor;p=1020.48,7950.65");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1575.96,7929.24");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2789.94,7956.83");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4751.02,7847.65");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=518.20,7588.61");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3156.62,7667.59");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4176.94,7542.38");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5784.81,7686.35");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1182.84,7370.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1567.71,7340.34");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2056.85,7502.77");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2695.16,7417.14");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4838.06,7325.76");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5317.69,7394.20");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=724.94,7164.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3156.18,7028.79");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3626.35,7242.74");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4303.01,7176.14");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5654.74,7103.83");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=363.25,6999.15");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1526.12,6771.17");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2084.01,6999.59");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2697.18,6857.17");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3595.74,6777.69");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4001.80,6936.35");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4596.76,6825.56");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5075.14,6895.73");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=240.07,6622.69");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=927.04,6699.29");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2528.13,6535.03");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4164.04,6622.09");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=365.77,6290.61");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2135.26,6514.81");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3069.19,6485.70");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4624.94,6414.40");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4992.86,6371.67");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5464.95,6331.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5779.70,6492.04");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=987.09,6235.97");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1552.18,6154.47");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2007.25,6101.63");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2653.70,6045.23");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3165.51,6123.18");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3805.93,6244.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4250.77,6191.97");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5982.62,6152.64");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=30.80,5895.04");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1285.83,5877.98");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3600.78,5956.72");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4724.81,5988.13");
    }

}

